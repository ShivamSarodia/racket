"""Decode BLE packets and publish complete, timestamped IMU samples."""

from __future__ import annotations

from dataclasses import dataclass
import struct
import time
from typing import Dict, Optional, Tuple

from .events import (
    ClockDiagnostic,
    ImuConsumer,
    ImuSample,
    PacketDiagnostic,
    TimingDiagnostic,
    Vector3,
)


# BLE notification format:
#   bytes 0..3     uint32 packet sequence, little-endian
#   bytes 4..end   raw 7-byte IMU FIFO records
PACKET_HEADER_SIZE = 4
FIFO_RECORD_SIZE = 7

# FIFO sensor tags after raw_tag >> 3.
TAG_GYRO = 0x01
TAG_LOW_G = 0x02
TAG_TIMESTAMP = 0x04
TAG_HIGH_G = 0x1D

LOW_G_SCALE = 0.000488  # g / LSB
HIGH_G_SCALE = 0.010417  # g / LSB
GYRO_SCALE = 0.140  # dps / LSB


@dataclass
class _Slot:
    index: int
    tag_counter: int
    low_g: Optional[Vector3] = None
    high_g: Optional[Vector3] = None
    gyro: Optional[Vector3] = None
    duplicate_sensor: bool = False

    @property
    def has_sensor_data(self) -> bool:
        return any((self.low_g, self.high_g, self.gyro))

    @property
    def is_complete(self) -> bool:
        return (
            not self.duplicate_sensor
            and self.low_g is not None
            and self.high_g is not None
            and self.gyro is not None
        )


class ImuPublisher:
    """Stateful decoder that delivers only complete, anchored samples."""

    def __init__(self, consumer: ImuConsumer):
        self._consumer = consumer

        self._expected_sequence: Optional[int] = None

        self._timestamp_tick_seconds: Optional[float] = None
        self._last_timestamp_raw: Optional[int] = None
        self._last_timestamp_unwrapped: Optional[int] = None
        self._timestamp_wraps = 0
        self._first_timestamp_unwrapped: Optional[int] = None
        self._anchors_seen = 0

        self._slots: Dict[int, _Slot] = {}
        self._current_slot: Optional[_Slot] = None
        self._previous_anchor: Optional[Tuple[int, int]] = None

    def configure_clock(self, freq_fine: int) -> None:
        """Configure conversion from hardware timestamp ticks to seconds."""

        correction = 1.0 + 0.0013 * freq_fine
        self._timestamp_tick_seconds = 1.0 / (46080.0 * correction)
        calculated_odr_hz = 960.0 * correction

        self._consumer.on_diagnostic(
            ClockDiagnostic(
                freq_fine=freq_fine,
                timestamp_tick_seconds=self._timestamp_tick_seconds,
                calculated_odr_hz=calculated_odr_hz,
            )
        )

    def process_notification(
        self,
        data: bytes,
        received_at: Optional[float] = None,
    ) -> None:
        """Decode one BLE notification and publish resulting events."""

        now = time.monotonic() if received_at is None else received_at
        data = bytes(data)

        if len(data) < PACKET_HEADER_SIZE:
            self._expected_sequence = None
            self._reset_reconstruction("packet shorter than sequence header")
            self._emit_bad_packet(now, None, "packet shorter than sequence header")
            return

        sequence = struct.unpack_from("<I", data, 0)[0]
        fifo_bytes = len(data) - PACKET_HEADER_SIZE

        if fifo_bytes % FIFO_RECORD_SIZE != 0:
            self._expected_sequence = (sequence + 1) & 0xFFFFFFFF
            self._reset_reconstruction("packet contains a partial FIFO record")
            self._emit_bad_packet(
                now,
                sequence,
                "packet contains a partial FIFO record",
            )
            return

        missing_packets = 0

        if self._expected_sequence is not None and sequence != self._expected_sequence:
            delta = (sequence - self._expected_sequence) & 0xFFFFFFFF

            if delta < 0x80000000:
                missing_packets = delta
                self._reset_reconstruction(
                    "missing {} BLE packet{}".format(
                        delta,
                        "" if delta == 1 else "s",
                    )
                )
            else:
                self._reset_reconstruction(
                    "duplicate or out-of-order BLE packet"
                )
                self._consumer.on_diagnostic(
                    PacketDiagnostic(
                        received_at=now,
                        sequence=sequence,
                        framing_valid=True,
                        accepted=False,
                        error="duplicate or out-of-order packet",
                    )
                )
                return

        self._expected_sequence = (sequence + 1) & 0xFFFFFFFF

        counts = {
            "gyro": 0,
            "low_g": 0,
            "high_g": 0,
            "timestamp": 0,
            "other": 0,
        }

        for offset in range(
            PACKET_HEADER_SIZE,
            len(data),
            FIFO_RECORD_SIZE,
        ):
            record = data[offset:offset + FIFO_RECORD_SIZE]
            record_kind = self._process_fifo_record(record)
            counts[record_kind] += 1

        self._consumer.on_diagnostic(
            PacketDiagnostic(
                received_at=now,
                sequence=sequence,
                framing_valid=True,
                accepted=True,
                missing_packets=missing_packets,
                total_records=sum(counts.values()),
                gyro_records=counts["gyro"],
                low_g_records=counts["low_g"],
                high_g_records=counts["high_g"],
                timestamp_records=counts["timestamp"],
                other_records=counts["other"],
            )
        )

    def close(self) -> None:
        """Discard records that never received a closing timestamp anchor."""

        self._reset_reconstruction("stream ended before a closing anchor")

    def _emit_bad_packet(
        self,
        received_at: float,
        sequence: Optional[int],
        error: str,
    ) -> None:
        self._consumer.on_diagnostic(
            PacketDiagnostic(
                received_at=received_at,
                sequence=sequence,
                framing_valid=False,
                accepted=False,
                error=error,
            )
        )

    def _process_fifo_record(self, record: bytes) -> str:
        raw_tag = record[0]
        sensor_tag = raw_tag >> 3
        tag_counter = (raw_tag >> 1) & 0x03
        slot = self._get_slot(tag_counter)

        if sensor_tag == TAG_TIMESTAMP:
            raw_timestamp = struct.unpack_from("<I", record, 1)[0]
            self._process_anchor(slot, raw_timestamp)
            return "timestamp"

        if sensor_tag not in (TAG_GYRO, TAG_LOW_G, TAG_HIGH_G):
            return "other"

        x, y, z = struct.unpack_from("<hhh", record, 1)

        if sensor_tag == TAG_GYRO:
            self._store_sensor(
                slot,
                "gyro",
                Vector3(x * GYRO_SCALE, y * GYRO_SCALE, z * GYRO_SCALE),
            )
            return "gyro"

        if sensor_tag == TAG_LOW_G:
            self._store_sensor(
                slot,
                "low_g",
                Vector3(x * LOW_G_SCALE, y * LOW_G_SCALE, z * LOW_G_SCALE),
            )
            return "low_g"

        self._store_sensor(
            slot,
            "high_g",
            Vector3(x * HIGH_G_SCALE, y * HIGH_G_SCALE, z * HIGH_G_SCALE),
        )
        return "high_g"

    def _get_slot(self, tag_counter: int) -> _Slot:
        if self._current_slot is None:
            slot = _Slot(index=0, tag_counter=tag_counter)
            self._current_slot = slot
            self._slots[slot.index] = slot
            return slot

        if tag_counter == self._current_slot.tag_counter:
            return self._current_slot

        expected = (self._current_slot.tag_counter + 1) & 0x03

        if tag_counter != expected:
            self._reset_reconstruction(
                "inconsistent FIFO time-slot progression"
            )
            slot = _Slot(index=0, tag_counter=tag_counter)
        else:
            slot = _Slot(
                index=self._current_slot.index + 1,
                tag_counter=tag_counter,
            )

        self._current_slot = slot
        self._slots[slot.index] = slot
        return slot

    @staticmethod
    def _store_sensor(slot: _Slot, name: str, value: Vector3) -> None:
        if getattr(slot, name) is not None:
            slot.duplicate_sensor = True
            return

        setattr(slot, name, value)

    def _process_anchor(self, slot: _Slot, raw_timestamp: int) -> None:
        if self._timestamp_tick_seconds is None:
            self._reset_reconstruction(
                "timestamp received before clock calibration"
            )
            return

        unwrapped = self._unwrap_timestamp(raw_timestamp)

        if unwrapped is None:
            self._reset_reconstruction("hardware timestamp moved backwards")
            return

        self._anchors_seen += 1

        if self._first_timestamp_unwrapped is None:
            self._first_timestamp_unwrapped = unwrapped

        if self._previous_anchor is None:
            discarded = self._discard_slots_before(slot.index)
            self._previous_anchor = (slot.index, unwrapped)
            self._consumer.on_diagnostic(
                TimingDiagnostic(
                    message="timestamp reconstruction anchored",
                    anchors_seen=self._anchors_seen,
                    discarded_samples=discarded,
                    imu_elapsed_seconds=self._timestamp_seconds(unwrapped),
                )
            )
            return

        previous_slot_index, previous_ticks = self._previous_anchor
        slot_distance = slot.index - previous_slot_index

        if slot_distance <= 0 or unwrapped <= previous_ticks:
            self._reset_reconstruction("invalid timestamp anchor interval")
            replacement = _Slot(index=0, tag_counter=slot.tag_counter)
            self._current_slot = replacement
            self._slots[replacement.index] = replacement
            self._previous_anchor = (replacement.index, unwrapped)
            return

        anchor_spacing_seconds = (
            (unwrapped - previous_ticks) * self._timestamp_tick_seconds
        )
        sample_period_seconds = anchor_spacing_seconds / slot_distance

        for slot_index in range(previous_slot_index, slot.index):
            candidate = self._slots.get(slot_index)

            if candidate is None or not candidate.is_complete:
                self._consumer.on_diagnostic(
                    TimingDiagnostic(
                        message="incomplete FIFO time slot discarded",
                        anchors_seen=self._anchors_seen,
                        discarded_samples=1,
                        imu_elapsed_seconds=self._timestamp_seconds(unwrapped),
                        anchor_spacing_seconds=anchor_spacing_seconds,
                        sample_period_seconds=sample_period_seconds,
                    )
                )
                continue

            timestamp_seconds = (
                self._timestamp_seconds(previous_ticks)
                + (slot_index - previous_slot_index) * sample_period_seconds
            )
            assert candidate.low_g is not None
            assert candidate.high_g is not None
            assert candidate.gyro is not None
            self._consumer.on_sample(
                ImuSample(
                    timestamp_seconds=timestamp_seconds,
                    low_g=candidate.low_g,
                    high_g=candidate.high_g,
                    gyro=candidate.gyro,
                )
            )

        for slot_index in list(self._slots):
            if slot_index < slot.index:
                del self._slots[slot_index]

        self._previous_anchor = (slot.index, unwrapped)
        self._consumer.on_diagnostic(
            TimingDiagnostic(
                message="timestamp interval reconstructed",
                anchors_seen=self._anchors_seen,
                imu_elapsed_seconds=self._timestamp_seconds(unwrapped),
                anchor_spacing_seconds=anchor_spacing_seconds,
                sample_period_seconds=sample_period_seconds,
            )
        )

    def _unwrap_timestamp(self, raw_timestamp: int) -> Optional[int]:
        wraps = self._timestamp_wraps

        if self._last_timestamp_raw is not None:
            if (
                raw_timestamp < self._last_timestamp_raw
                and self._last_timestamp_raw - raw_timestamp > 0x80000000
            ):
                wraps += 1

        unwrapped = raw_timestamp + wraps * (1 << 32)

        if (
            self._last_timestamp_unwrapped is not None
            and unwrapped <= self._last_timestamp_unwrapped
        ):
            return None

        self._timestamp_wraps = wraps
        self._last_timestamp_raw = raw_timestamp
        self._last_timestamp_unwrapped = unwrapped
        return unwrapped

    def _timestamp_seconds(self, unwrapped_ticks: int) -> float:
        assert self._first_timestamp_unwrapped is not None
        assert self._timestamp_tick_seconds is not None
        return (
            unwrapped_ticks - self._first_timestamp_unwrapped
        ) * self._timestamp_tick_seconds

    def _discard_slots_before(self, slot_index: int) -> int:
        discarded = 0

        for candidate_index in list(self._slots):
            if candidate_index >= slot_index:
                continue
            candidate = self._slots.pop(candidate_index)
            if candidate.has_sensor_data:
                discarded += 1

        return discarded

    def _reset_reconstruction(self, reason: str) -> None:
        discarded = sum(
            1 for slot in self._slots.values() if slot.has_sensor_data
        )
        had_state = bool(self._slots or self._previous_anchor)

        self._slots.clear()
        self._current_slot = None
        self._previous_anchor = None

        if had_state or discarded:
            self._consumer.on_diagnostic(
                TimingDiagnostic(
                    message=reason,
                    anchors_seen=self._anchors_seen,
                    discarded_samples=discarded,
                )
            )
