"""Serial connection value object and basic syntax/range validation.

These are source defaults, not a readback of the commissioned drive. Persisted
GUI settings can override them. validate() checks plausibility only; it cannot
prove that the selected port, baud rate or slave corresponds to the rig.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class SerialConfig:
    port: str = "COM6"
    slave_address: int = 1
    baudrate: int = 115_200
    bytesize: int = 8
    parity: str = "N"
    stopbits: int = 2
    timeout_s: float = 1.0

    def validate(self) -> None:
        if not self.port.strip():
            raise ValueError("A serial port is required")
        if not 1 <= self.slave_address <= 247:
            raise ValueError("Slave address must be between 1 and 247")
        if self.baudrate <= 0:
            raise ValueError("Baud rate must be positive")
        if self.bytesize not in (7, 8):
            raise ValueError("Data bits must be 7 or 8")
        if self.parity not in ("N", "E", "O"):
            raise ValueError("Parity must be N, E, or O")
        if self.stopbits not in (1, 2):
            raise ValueError("Stop bits must be 1 or 2")
        if self.timeout_s <= 0:
            raise ValueError("Timeout must be positive")
