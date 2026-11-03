"""Optional AIN0 acquisition on an independent USB worker.

LJM is tried first, then the U3 LabJackPython backend. Neither consumes the
drive's Modbus link. Voltage samples use perf_counter after the USB read, not
a hardware latch timestamp. The linear conversion is (V - zero_V) * mm_per_V;
None scale means voltage-only acquisition. GUI association is a lookup into
this bounded history, not hardware synchronization with the encoder.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from threading import Event, Lock, Thread
from time import monotonic, perf_counter
from .scene_calibration import displacement_mm


@dataclass(frozen=True)
class AinSample:
    sampled_monotonic_s: float
    voltage_v: float
    extension_mm: float | None


class LabJackAin0:
    """Optional, independent AIN0 sampler using LabJack LJM.

    Sampling runs on its own USB thread, so it never consumes or blocks the
    servo drive's Modbus link. Position conversion is intentionally a simple,
    explicit two-point linear calibration.
    """

    def __init__(self, max_samples: int = 36_000) -> None:
        self.samples: deque[AinSample] = deque(maxlen=max_samples)
        self._lock = Lock()
        self._stop = Event()
        self._thread: Thread | None = None
        self._handle: int | None = None
        self._ljm = None
        self._u3_device = None
        self.backend = ""
        self.connected = False
        self.last_error: str | None = None
        self.sample_rate_hz = 100.0
        self.zero_voltage_v = 0.0
        self.mm_per_volt: float | None = None

    def start(
        self,
        *,
        sample_rate_hz: float = 100.0,
        zero_voltage_v: float = 0.0,
        mm_per_volt: float | None = None,
    ) -> None:
        if not 1.0 <= sample_rate_hz <= 1000.0:
            raise ValueError("LabJack sample rate must be between 1 and 1000 Hz")
        self.stop()
        self.sample_rate_hz = sample_rate_hz
        self.zero_voltage_v = zero_voltage_v
        self.mm_per_volt = mm_per_volt
        self.last_error = None
        self._stop.clear()
        ljm_error: Exception | None = None
        try:
            from labjack import ljm

            self._ljm = ljm
            self._handle = ljm.openS("ANY", "ANY", "ANY")
            ljm.eWriteName(self._handle, "AIN0_NEGATIVE_CH", 199)
            ljm.eWriteName(self._handle, "AIN0_RANGE", 10.0)
            self.backend = "LJM"
        except Exception as exc:
            ljm_error = exc
            self._handle = None
            self._ljm = None
            try:
                import u3

                self._u3_device = u3.U3()
                self._u3_device.configAnalog(0)
                self.backend = "U3 / LabJackPython"
            except Exception as u3_exc:
                raise RuntimeError(
                    f"LJM connection failed: {ljm_error}; U3 connection failed: {u3_exc}"
                ) from u3_exc
        self.connected = True
        self._thread = Thread(target=self._run, daemon=True, name="labjack-ain0")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=0.5)
        self._thread = None
        handle, ljm = self._handle, self._ljm
        self._handle = None
        self.connected = False
        if handle is not None and ljm is not None:
            try:
                ljm.close(handle)
            except Exception:
                pass
        if self._u3_device is not None:
            try:
                self._u3_device.close()
            except Exception:
                pass
        self._u3_device = None
        self.backend = ""

    def clear(self) -> None:
        with self._lock:
            self.samples.clear()

    def latest_before(self, sampled_at: float) -> AinSample | None:
        # Compatibility caveat: if every sample is later than sampled_at, this
        # returns the oldest sample anyway. Callers must not interpret this fallback
        # as guaranteed causal alignment; no maximum-age check is applied here.
        with self._lock:
            for sample in reversed(self.samples):
                if sample.sampled_monotonic_s <= sampled_at:
                    return sample
            return self.samples[0] if self.samples else None

    def _run(self) -> None:
        # Use monotonic only to pace this worker; stamp acquired samples with
        # perf_counter so GUI association shares the encoder acquisition domain.
        if (self._ljm is None or self._handle is None) and self._u3_device is None:
            raise RuntimeError("No LabJack backend is connected")
        period = 1.0 / self.sample_rate_hz
        deadline = monotonic()
        while not self._stop.is_set():
            try:
                voltage = (
                    float(self._u3_device.getAIN(0))
                    if self._u3_device is not None
                    else float(self._ljm.eReadName(self._handle, "AIN0"))
                )
                sampled_at = perf_counter()
                extension = displacement_mm(voltage, self.zero_voltage_v, self.mm_per_volt)
                with self._lock:
                    self.samples.append(AinSample(sampled_at, voltage, extension))
            except Exception as exc:
                self.last_error = str(exc)
                self.connected = False
                break
            deadline += period
            remaining = deadline - monotonic()
            if remaining < -period:
                deadline = monotonic()
                remaining = 0.0
            self._stop.wait(max(0.0, remaining))
