"""NTP-style UDP clock exchange for PC <-> LattePanda test alignment.

This module estimates the offset between the two computers' high-resolution
``perf_counter_ns()`` clocks. It does not change either Windows system clock.

It is compatible with the standalone LattePanda responder using:
    clock UDP port      49152
    discovery UDP port  49153


Maintenance contract: local means Motor PC; remote means LattePanda.
Protocol structures must stay byte-compatible with the standalone responder.
UTC is diagnostic only. TestRunClock freezes one offset so later replies cannot
retroactively move exported samples; it does not estimate PC clock drift over
the run. A low RTT reduces exposure to latency but does not prove symmetry.
"""

from __future__ import annotations

import csv
import socket
import struct
import threading
import uuid
from dataclasses import dataclass
from pathlib import Path
from time import monotonic, perf_counter_ns, time_ns
from typing import Callable


MAGIC = b"STRG"
VERSION = 1

DEFAULT_PORT = 49152
DISCOVERY_PORT = 49153

REQUEST = struct.Struct("!4sBQQQ")
RESPONSE = struct.Struct("!4sBQQQQQQQ")

DISCOVERY_REQUEST = b"STRG_DISCOVER_V1"
DISCOVERY_RESPONSE_PREFIX = "STRG_RESPONDER_V1"


@dataclass(frozen=True)
class DiscoveredResponder:
    ip: str
    hostname: str
    clock_port: int


@dataclass(frozen=True)
class ClockSyncSample:
    sequence: int
    local_send_monotonic_ns: int
    remote_receive_monotonic_ns: int
    remote_send_monotonic_ns: int
    local_receive_monotonic_ns: int
    local_send_utc_ns: int
    remote_receive_utc_ns: int
    remote_send_utc_ns: int
    local_receive_utc_ns: int

    @property
    def round_trip_ns(self) -> int:
        """Estimated network round-trip time with responder processing removed."""
        return max(
            0,
            (self.local_receive_monotonic_ns - self.local_send_monotonic_ns)
            - (self.remote_send_monotonic_ns - self.remote_receive_monotonic_ns),
        )

    @property
    def local_midpoint_monotonic_ns(self) -> float:
        """PC monotonic time at the midpoint of the network exchange."""
        return (
            self.local_send_monotonic_ns + self.local_receive_monotonic_ns
        ) / 2.0

    @property
    def remote_minus_local_ns(self) -> float:
        """Estimated offset: remote perf_counter_ns - local perf_counter_ns.

        A positive value means the LattePanda perf-counter reading is ahead
        of the PC perf-counter reading by this amount at the same instant.

        The counters do not need the same zero/epoch. This NTP-style equation
        estimates the difference between their arbitrary origins.
        """
        return (
            (self.remote_receive_monotonic_ns - self.local_send_monotonic_ns)
            + (self.remote_send_monotonic_ns - self.local_receive_monotonic_ns)
        ) / 2.0

    @property
    def remote_minus_local_utc_ns(self) -> float:
        """Estimated LattePanda wall-clock offset relative to the PC wall clock."""
        return (
            (self.remote_receive_utc_ns - self.local_send_utc_ns)
            + (self.remote_send_utc_ns - self.local_receive_utc_ns)
        ) / 2.0

    def remote_to_local_monotonic_ns(self, remote_ns: int | float) -> float:
        """Convert a LattePanda perf-counter timestamp into the PC clock domain."""
        return float(remote_ns) - self.remote_minus_local_ns

    def local_to_remote_monotonic_ns(self, local_ns: int | float) -> float:
        """Convert a PC perf-counter timestamp into the LattePanda clock domain."""
        return float(local_ns) + self.remote_minus_local_ns


@dataclass(frozen=True)
class TestRunClock:
    """Frozen PC-to-LattePanda clock mapping for one motion-profile run.

    A run deliberately retains the one synchronization observation selected at
    T0.  Later UDP replies must not change previously exported timestamps.
    """

    run_id: str
    source_profile_path: Path
    motor_pc_t0_monotonic_ns: int
    lattepanda_t0_monotonic_ns: int | None
    remote_minus_local_ns: float | None
    clock_round_trip_ns: int | None
    clock_sync_sequence: int | None

    @classmethod
    def create(
        cls,
        source_profile_path: str | Path,
        motor_pc_t0_monotonic_ns: int,
        sync_sample: ClockSyncSample | None,
        run_id: str | None = None,
    ) -> "TestRunClock":
        offset = None if sync_sample is None else sync_sample.remote_minus_local_ns
        return cls(
            run_id=run_id or uuid.uuid4().hex,
            source_profile_path=Path(source_profile_path),
            motor_pc_t0_monotonic_ns=motor_pc_t0_monotonic_ns,
            lattepanda_t0_monotonic_ns=(
                None
                if offset is None
                else round(motor_pc_t0_monotonic_ns + offset)
            ),
            remote_minus_local_ns=offset,
            clock_round_trip_ns=(
                None if sync_sample is None else sync_sample.round_trip_ns
            ),
            clock_sync_sequence=(
                None if sync_sample is None else sync_sample.sequence
            ),
        )

    @classmethod
    def from_scheduled_t0(
        cls,
        source_profile_path: str | Path,
        lattepanda_t0_monotonic_ns: int,
        sync_sample: ClockSyncSample,
        run_id: str,
    ) -> "TestRunClock":
        return cls(
            run_id=run_id,
            source_profile_path=Path(source_profile_path),
            motor_pc_t0_monotonic_ns=round(
                sync_sample.remote_to_local_monotonic_ns(lattepanda_t0_monotonic_ns)
            ),
            lattepanda_t0_monotonic_ns=lattepanda_t0_monotonic_ns,
            remote_minus_local_ns=sync_sample.remote_minus_local_ns,
            clock_round_trip_ns=sync_sample.round_trip_ns,
            clock_sync_sequence=sync_sample.sequence,
        )

    @property
    def synchronized(self) -> bool:
        return self.lattepanda_t0_monotonic_ns is not None

    def motor_pc_to_lattepanda_ns(self, motor_pc_ns: int) -> int | None:
        if self.remote_minus_local_ns is None:
            return None
        return round(motor_pc_ns + self.remote_minus_local_ns)

    def latte_panda_time_s(self, motor_pc_ns: int) -> float | None:
        """Return sample time relative to LattePanda T0."""
        remote_ns = self.motor_pc_to_lattepanda_ns(motor_pc_ns)
        if remote_ns is None or self.lattepanda_t0_monotonic_ns is None:
            return None
        return (remote_ns - self.lattepanda_t0_monotonic_ns) / 1_000_000_000.0


def discover_lattepanda(
    timeout_s: float = 2.0,
    discovery_port: int = DISCOVERY_PORT,
    expected_hostname: str | None = None,
) -> DiscoveredResponder:
    """Discover the LattePanda responder by UDP broadcast.

    The responder replies with:
        STRG_RESPONDER_V1|<hostname>|<clock_port>

    ``expected_hostname`` may be set to e.g. ``DESKTOP-IBIHN8H`` to ignore
    replies from any other compatible responder.

    Raises TimeoutError if no responder is found. Some managed Wi-Fi networks
    block client-to-client broadcast; in that case pass the current IP address
    directly to ``UdpClockSyncClient.start()``.
    """
    if timeout_s <= 0:
        raise ValueError("Discovery timeout must be greater than zero")
    if not 1 <= discovery_port <= 65535:
        raise ValueError("Discovery UDP port must be between 1 and 65535")

    expected = expected_hostname.casefold() if expected_hostname else None

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.settimeout(timeout_s)

        # Bind an ephemeral local port so the responder can reply directly.
        sock.bind(("", 0))
        sock.sendto(DISCOVERY_REQUEST, ("255.255.255.255", discovery_port))

        deadline = monotonic() + timeout_s

        while True:
            remaining = deadline - monotonic()
            if remaining <= 0:
                raise TimeoutError("No LattePanda clock responder discovered")

            sock.settimeout(remaining)

            try:
                packet, address = sock.recvfrom(512)
            except socket.timeout as exc:
                raise TimeoutError(
                    "No LattePanda clock responder discovered"
                ) from exc

            try:
                text = packet.decode("utf-8")
                prefix, hostname, port_text = text.split("|", 2)
                clock_port = int(port_text)
            except (UnicodeDecodeError, ValueError):
                continue

            if prefix != DISCOVERY_RESPONSE_PREFIX:
                continue
            if not 1 <= clock_port <= 65535:
                continue
            if expected is not None and hostname.casefold() != expected:
                continue

            return DiscoveredResponder(
                ip=address[0],
                hostname=hostname,
                clock_port=clock_port,
            )


class UdpClockSyncClient:
    def __init__(
        self,
        on_sample: Callable[[ClockSyncSample], None] | None = None,
        on_error: Callable[[str], None] | None = None,
    ) -> None:
        self.on_sample = on_sample
        self.on_error = on_error
        self._samples: list[ClockSyncSample] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

        self.host = ""
        self.port = DEFAULT_PORT
        self.rate_hz = 2.0
        self.resolved_ip = ""

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(
        self,
        host: str,
        port: int = DEFAULT_PORT,
        rate_hz: float = 2.0,
    ) -> None:
        """Start syncing to a hostname/IP, or use ``host="auto"`` for discovery."""
        host = host.strip()

        if not host:
            raise ValueError(
                'A LattePanda hostname/IP is required, or use host="auto"'
            )
        if not 1 <= port <= 65535:
            raise ValueError("UDP port must be between 1 and 65535")
        if not 0.2 <= rate_hz <= 20.0:
            raise ValueError("Clock exchange rate must be between 0.2 and 20 Hz")

        self.stop()

        if host.casefold() == "auto":
            responder = discover_lattepanda(
                expected_hostname="DESKTOP-IBIHN8H",
            )
            self.host = responder.ip
            self.port = responder.clock_port
        else:
            self.host = host
            self.port = port

        self.rate_hz = rate_hz
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            daemon=True,
            name="udp-clock-sync",
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self._thread = None

    def clear(self) -> None:
        with self._lock:
            self._samples.clear()

    def samples(self) -> list[ClockSyncSample]:
        with self._lock:
            return list(self._samples)

    def latest_sample(self) -> ClockSyncSample | None:
        samples = self.samples()
        return samples[-1] if samples else None

    def best_sample(self) -> ClockSyncSample | None:
        """Return the minimum-RTT sample from the full session."""
        samples = self.samples()
        return min(samples, key=lambda item: item.round_trip_ns) if samples else None

    def best_recent_sample(self, count: int = 20) -> ClockSyncSample | None:
        """Return the minimum-RTT sample among the most recent samples.

        This is preferable for a long test because oscillator drift makes an
        old low-RTT sample less representative of the current clock offset.
        """
        if count < 1:
            raise ValueError("count must be at least 1")
        samples = self.samples()
        recent = samples[-count:]
        return min(recent, key=lambda item: item.round_trip_ns) if recent else None

    def export_csv(
        self,
        path: str | Path,
        samples: list[ClockSyncSample] | None = None,
    ) -> None:
        # These are exchange diagnostics; the run CSV separately repeats the one
        # frozen offset used for that run. Do not remap samples using later replies.
        with Path(path).open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow((
                "sequence",
                "local_send_monotonic_ns",
                "remote_receive_monotonic_ns",
                "remote_send_monotonic_ns",
                "local_receive_monotonic_ns",
                "round_trip_ms",
                "remote_minus_local_ms",
                "remote_minus_local_utc_ms",
                "local_send_utc_ns",
                "remote_receive_utc_ns",
                "remote_send_utc_ns",
                "local_receive_utc_ns",
            ))

            export_samples = self.samples() if samples is None else samples
            for sample in export_samples:
                writer.writerow((
                    sample.sequence,
                    sample.local_send_monotonic_ns,
                    sample.remote_receive_monotonic_ns,
                    sample.remote_send_monotonic_ns,
                    sample.local_receive_monotonic_ns,
                    f"{sample.round_trip_ns / 1_000_000.0:.6f}",
                    f"{sample.remote_minus_local_ns / 1_000_000.0:.6f}",
                    f"{sample.remote_minus_local_utc_ns / 1_000_000.0:.6f}",
                    sample.local_send_utc_ns,
                    sample.remote_receive_utc_ns,
                    sample.remote_send_utc_ns,
                    sample.local_receive_utc_ns,
                ))

    def _run(self) -> None:
        sequence = 0
        period_s = 1.0 / self.rate_hz
        next_deadline = monotonic()

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(min(0.75, max(0.15, period_s * 0.8)))

            try:
                target_ip = socket.gethostbyname(self.host)
            except OSError as exc:
                if self.on_error is not None:
                    self.on_error(f"Could not resolve LattePanda host {self.host!r}: {exc}")
                return

            self.resolved_ip = target_ip
            target = (target_ip, self.port)

            while not self._stop.is_set():
                sequence += 1

                try:
                    local_send_mono = perf_counter_ns()
                    local_send_utc = time_ns()

                    request = REQUEST.pack(
                        MAGIC,
                        VERSION,
                        sequence,
                        local_send_mono,
                        local_send_utc,
                    )

                    sock.sendto(request, target)

                    # Ignore unrelated UDP packets until this exchange times out.
                    while True:
                        response, address = sock.recvfrom(65535)
                        if address[0] != target_ip or address[1] != self.port:
                            continue
                        break

                    local_receive_mono = perf_counter_ns()
                    local_receive_utc = time_ns()

                    if len(response) != RESPONSE.size:
                        raise ValueError(
                            f"invalid clock response size: "
                            f"{len(response)} bytes, expected {RESPONSE.size}"
                        )

                    unpacked = RESPONSE.unpack(response)
                    magic, version, reply_sequence = unpacked[:3]

                    if (
                        magic != MAGIC
                        or version != VERSION
                        or reply_sequence != sequence
                    ):
                        raise ValueError(
                            "invalid or out-of-sequence clock response"
                        )

                    # The responder echoes the exact original PC timestamps.
                    if (
                        unpacked[3] != local_send_mono
                        or unpacked[4] != local_send_utc
                    ):
                        raise ValueError(
                            "clock response did not echo the original request"
                        )

                    sample = ClockSyncSample(
                        sequence=sequence,
                        local_send_monotonic_ns=local_send_mono,
                        local_send_utc_ns=local_send_utc,
                        remote_receive_monotonic_ns=unpacked[5],
                        remote_receive_utc_ns=unpacked[6],
                        remote_send_monotonic_ns=unpacked[7],
                        remote_send_utc_ns=unpacked[8],
                        local_receive_monotonic_ns=local_receive_mono,
                        local_receive_utc_ns=local_receive_utc,
                    )

                    with self._lock:
                        self._samples.append(sample)

                    if self.on_sample is not None:
                        self.on_sample(sample)

                except (OSError, ValueError, struct.error) as exc:
                    if not self._stop.is_set() and self.on_error is not None:
                        self.on_error(str(exc))

                next_deadline += period_s
                remaining = next_deadline - monotonic()

                if remaining < -period_s:
                    next_deadline = monotonic()
                    remaining = 0.0

                self._stop.wait(max(0.0, remaining))


# Retained for backwards compatibility if you ever want to run this same
# module as a responder. The standalone LattePanda script is preferred.
def serve_clock(host: str = "0.0.0.0", port: int = DEFAULT_PORT) -> None:
    """Run a compatible clock responder until interrupted."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((host, port))

        while True:
            packet, address = sock.recvfrom(REQUEST.size)
            remote_receive_mono = perf_counter_ns()
            remote_receive_utc = time_ns()

            try:
                magic, version, sequence, local_send_mono, local_send_utc = (
                    REQUEST.unpack(packet)
                )
            except struct.error:
                continue

            if magic != MAGIC or version != VERSION:
                continue

            remote_send_mono = perf_counter_ns()
            remote_send_utc = time_ns()

            sock.sendto(
                RESPONSE.pack(
                    MAGIC,
                    VERSION,
                    sequence,
                    local_send_mono,
                    local_send_utc,
                    remote_receive_mono,
                    remote_receive_utc,
                    remote_send_mono,
                    remote_send_utc,
                ),
                address,
            )
