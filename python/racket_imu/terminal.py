"""Terminal dashboard implemented as a public IMU consumer."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
import sys
import time
from typing import Optional, TextIO

from .events import (
    ClockDiagnostic,
    ConnectionDiagnostic,
    ConnectionState,
    Diagnostic,
    ImuConsumer,
    ImuSample,
    PacketDiagnostic,
    TimingDiagnostic,
    Vector3,
)


def _zero_vector() -> Vector3:
    return Vector3(0.0, 0.0, 0.0)


def _magnitude(value: Vector3) -> float:
    return (
        value.x * value.x
        + value.y * value.y
        + value.z * value.z
    ) ** 0.5


@dataclass
class _DashboardState:
    low_g: Vector3 = field(default_factory=_zero_vector)
    high_g: Vector3 = field(default_factory=_zero_vector)
    gyro: Vector3 = field(default_factory=_zero_vector)

    samples_published: int = 0
    discarded_samples: int = 0

    low_g_records: int = 0
    high_g_records: int = 0
    gyro_records: int = 0
    timestamp_records: int = 0
    other_records: int = 0
    total_records: int = 0

    packets_received: int = 0
    missing_packets: int = 0
    bad_packets: int = 0
    first_packet_time: Optional[float] = None
    last_packet_time: Optional[float] = None

    freq_fine: Optional[int] = None
    timestamp_tick_seconds: Optional[float] = None
    calculated_odr_hz: Optional[float] = None
    imu_elapsed_seconds: Optional[float] = None
    last_anchor_spacing_seconds: Optional[float] = None
    reconstructed_period_seconds: Optional[float] = None
    last_timing_message: Optional[str] = None


class TerminalConsumer(ImuConsumer):
    """Maintain and render the live terminal dashboard from public events."""

    def __init__(self, output: TextIO = sys.stdout):
        self._output = output
        self._state = _DashboardState()
        self._streaming = False

    def on_sample(self, sample: ImuSample) -> None:
        self._state.low_g = sample.low_g
        self._state.high_g = sample.high_g
        self._state.gyro = sample.gyro
        self._state.samples_published += 1
        self._state.imu_elapsed_seconds = sample.timestamp_seconds

    def on_diagnostic(self, diagnostic: Diagnostic) -> None:
        if isinstance(diagnostic, ConnectionDiagnostic):
            self._handle_connection(diagnostic)
        elif isinstance(diagnostic, ClockDiagnostic):
            self._state.freq_fine = diagnostic.freq_fine
            self._state.timestamp_tick_seconds = (
                diagnostic.timestamp_tick_seconds
            )
            self._state.calculated_odr_hz = diagnostic.calculated_odr_hz
        elif isinstance(diagnostic, PacketDiagnostic):
            self._handle_packet(diagnostic)
        elif isinstance(diagnostic, TimingDiagnostic):
            self._state.discarded_samples += diagnostic.discarded_samples
            self._state.last_timing_message = diagnostic.message

            if diagnostic.imu_elapsed_seconds is not None:
                self._state.imu_elapsed_seconds = diagnostic.imu_elapsed_seconds
            if diagnostic.anchor_spacing_seconds is not None:
                self._state.last_anchor_spacing_seconds = (
                    diagnostic.anchor_spacing_seconds
                )
            if diagnostic.sample_period_seconds is not None:
                self._state.reconstructed_period_seconds = (
                    diagnostic.sample_period_seconds
                )

    def _handle_connection(self, diagnostic: ConnectionDiagnostic) -> None:
        if diagnostic.state == ConnectionState.SCANNING:
            self._write_line("Scanning for {}...".format(diagnostic.detail))
        elif diagnostic.state == ConnectionState.CONNECTING:
            self._write_line("Connecting to {}...".format(diagnostic.detail))
        elif diagnostic.state == ConnectionState.CONNECTED:
            self._write_line("Connected.")
        elif diagnostic.state == ConnectionState.STREAMING:
            self._streaming = True
            self._output.write("\033[2J\033[H")
            self._output.flush()
        elif diagnostic.state == ConnectionState.DISCONNECTED:
            self._streaming = False
            if diagnostic.detail:
                self._write_line(diagnostic.detail)

    def _handle_packet(self, diagnostic: PacketDiagnostic) -> None:
        if not diagnostic.framing_valid:
            self._state.bad_packets += 1
            return

        self._state.packets_received += 1
        self._state.missing_packets += diagnostic.missing_packets
        self._state.last_packet_time = diagnostic.received_at

        if self._state.first_packet_time is None:
            self._state.first_packet_time = diagnostic.received_at

        self._state.total_records += diagnostic.total_records
        self._state.gyro_records += diagnostic.gyro_records
        self._state.low_g_records += diagnostic.low_g_records
        self._state.high_g_records += diagnostic.high_g_records
        self._state.timestamp_records += diagnostic.timestamp_records
        self._state.other_records += diagnostic.other_records

    def _write_line(self, text: str) -> None:
        self._output.write(text + "\n")
        self._output.flush()

    async def render_forever(self) -> None:
        while True:
            if self._streaming:
                self.render()
            await asyncio.sleep(0.1)

    def render(self, now: Optional[float] = None) -> None:
        now = time.monotonic() if now is None else now
        state = self._state

        if state.first_packet_time is not None and now > state.first_packet_time:
            elapsed = now - state.first_packet_time
            fifo_rate = state.total_records / elapsed
            gyro_rate = state.gyro_records / elapsed
            low_g_rate = state.low_g_records / elapsed
            high_g_rate = state.high_g_records / elapsed
            timestamp_rate = state.timestamp_records / elapsed
            sample_rate = state.samples_published / elapsed
        else:
            fifo_rate = 0.0
            gyro_rate = 0.0
            low_g_rate = 0.0
            high_g_rate = 0.0
            timestamp_rate = 0.0
            sample_rate = 0.0

        packet_age_ms = None
        if state.last_packet_time is not None:
            packet_age_ms = (now - state.last_packet_time) * 1000.0

        lines = [
            "\033[H",
            "Racket IMU — Live",
            "=================",
            "",
            "{:14}{:>10}{:>10}{:>10}".format("", "X", "Y", "Z"),
            "{:14}{:10.3f}{:10.3f}{:10.3f}".format(
                "Low-g [g]", state.low_g.x, state.low_g.y, state.low_g.z
            ),
            "{:14}{:10.3f}{:10.3f}{:10.3f}".format(
                "High-g [g]", state.high_g.x, state.high_g.y, state.high_g.z
            ),
            "{:14}{:10.1f}{:10.1f}{:10.1f}".format(
                "Gyro [dps]", state.gyro.x, state.gyro.y, state.gyro.z
            ),
            "",
            "|low-g|       {:8.3f} g".format(_magnitude(state.low_g)),
            "|high-g|      {:8.3f} g".format(_magnitude(state.high_g)),
            "|gyro|        {:8.1f} dps".format(_magnitude(state.gyro)),
            "",
            "Transport",
            "---------",
            "Packets received:  {}".format(state.packets_received),
            "Missing packets:   {}".format(state.missing_packets),
            "Bad packets:       {}".format(state.bad_packets),
            "FIFO records/sec:  {:.1f}".format(fifo_rate),
            "",
            "gyro records:      {}   ({:.1f}/s)".format(
                state.gyro_records, gyro_rate
            ),
            "low-g records:     {}   ({:.1f}/s)".format(
                state.low_g_records, low_g_rate
            ),
            "high-g records:    {}   ({:.1f}/s)".format(
                state.high_g_records, high_g_rate
            ),
            "timestamp records: {}   ({:.1f}/s)".format(
                state.timestamp_records, timestamp_rate
            ),
            "complete samples:  {}   ({:.1f}/s)".format(
                state.samples_published, sample_rate
            ),
            "discarded samples: {}".format(state.discarded_samples),
            "",
            "IMU timing",
            "----------",
        ]

        if state.freq_fine is None:
            lines.append("FREQ_FINE:         unavailable")
        else:
            assert state.timestamp_tick_seconds is not None
            assert state.calculated_odr_hz is not None
            lines.extend(
                [
                    "FREQ_FINE:         {}".format(state.freq_fine),
                    "Timestamp tick:    {:.4f} us".format(
                        state.timestamp_tick_seconds * 1e6
                    ),
                    "Calculated ODR:    {:.3f} Hz".format(
                        state.calculated_odr_hz
                    ),
                    "Expected TS rate:  {:.3f} Hz".format(
                        state.calculated_odr_hz / 8.0
                    ),
                ]
            )

        if state.imu_elapsed_seconds is None:
            lines.append("IMU elapsed time:  waiting for timestamps...")
        else:
            lines.append(
                "IMU elapsed time:  {:.6f} s".format(
                    state.imu_elapsed_seconds
                )
            )

        if state.last_anchor_spacing_seconds is not None:
            lines.append(
                "Last TS spacing:   {:.3f} ms".format(
                    state.last_anchor_spacing_seconds * 1000.0
                )
            )

        if state.reconstructed_period_seconds is not None:
            lines.append(
                "Sample spacing:    {:.3f} ms".format(
                    state.reconstructed_period_seconds * 1000.0
                )
            )

        lines.append("")
        if packet_age_ms is None:
            lines.append("Last BLE packet:   waiting...")
        else:
            lines.append(
                "Last BLE packet:   {:.1f} ms ago".format(packet_age_ms)
            )

        lines.extend(["", "Ctrl-C to quit.", ""])
        self._output.write("\n".join(lines))
        self._output.flush()
