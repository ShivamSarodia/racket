"""Public events and consumer interface for the racket IMU stream."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Union


@dataclass(frozen=True)
class Vector3:
    """A three-axis measurement."""

    x: float
    y: float
    z: float


@dataclass(frozen=True)
class ImuSample:
    """One complete hardware sampling slot, relative to the first anchor.

    Accelerometer vectors are measured in g; the gyroscope is measured in dps.
    """

    timestamp_seconds: float
    low_g: Vector3
    high_g: Vector3
    gyro: Vector3


class ConnectionState(Enum):
    SCANNING = "scanning"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    STREAMING = "streaming"
    DISCONNECTED = "disconnected"


@dataclass(frozen=True)
class ConnectionDiagnostic:
    state: ConnectionState
    detail: Optional[str] = None


@dataclass(frozen=True)
class ClockDiagnostic:
    freq_fine: int
    timestamp_tick_seconds: float
    calculated_odr_hz: float


@dataclass(frozen=True)
class PacketDiagnostic:
    """Results for one received BLE notification."""

    received_at: float
    sequence: Optional[int]
    framing_valid: bool
    accepted: bool
    missing_packets: int = 0
    total_records: int = 0
    gyro_records: int = 0
    low_g_records: int = 0
    high_g_records: int = 0
    timestamp_records: int = 0
    other_records: int = 0
    error: Optional[str] = None


@dataclass(frozen=True)
class TimingDiagnostic:
    """Timestamp reconstruction progress or a recoverable warning."""

    message: str
    anchors_seen: int
    discarded_samples: int = 0
    imu_elapsed_seconds: Optional[float] = None
    anchor_spacing_seconds: Optional[float] = None
    sample_period_seconds: Optional[float] = None


Diagnostic = Union[
    ConnectionDiagnostic,
    ClockDiagnostic,
    PacketDiagnostic,
    TimingDiagnostic,
]


class ImuConsumer:
    """Receives complete samples and, optionally, stream diagnostics."""

    def on_sample(self, sample: ImuSample) -> None:
        raise NotImplementedError

    def on_diagnostic(self, diagnostic: Diagnostic) -> None:
        pass
