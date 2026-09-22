import struct
import unittest

from racket_imu import (
    ClockDiagnostic,
    ImuConsumer,
    PacketDiagnostic,
    TimingDiagnostic,
)
from racket_imu.publisher import (
    GYRO_SCALE,
    HIGH_G_SCALE,
    LOW_G_SCALE,
    ImuPublisher,
    TAG_GYRO,
    TAG_HIGH_G,
    TAG_LOW_G,
    TAG_TIMESTAMP,
)


def sensor_record(sensor_tag, tag_counter, x, y, z):
    raw_tag = (sensor_tag << 3) | (tag_counter << 1)
    return struct.pack("<Bhhh", raw_tag, x, y, z)


def timestamp_record(tag_counter, raw_timestamp):
    raw_tag = (TAG_TIMESTAMP << 3) | (tag_counter << 1)
    return struct.pack("<BIH", raw_tag, raw_timestamp, 0)


def complete_slot(index):
    tag_counter = index & 0x03
    value = index + 1
    return [
        sensor_record(TAG_GYRO, tag_counter, value, -value, value * 2),
        sensor_record(TAG_LOW_G, tag_counter, value, -value, value * 2),
        sensor_record(TAG_HIGH_G, tag_counter, value, -value, value * 2),
    ]


def packet(sequence, records):
    return struct.pack("<I", sequence) + b"".join(records)


class RecordingConsumer(ImuConsumer):
    def __init__(self):
        self.samples = []
        self.diagnostics = []

    def on_sample(self, sample):
        self.samples.append(sample)

    def on_diagnostic(self, diagnostic):
        self.diagnostics.append(diagnostic)


class MinimalConsumer(ImuConsumer):
    def __init__(self):
        self.samples = []

    def on_sample(self, sample):
        self.samples.append(sample)


def anchored_records(first_raw=1000, tick_distance=384):
    records = [timestamp_record(0, first_raw)]
    for index in range(8):
        records.extend(complete_slot(index))
    records.append(timestamp_record(0, (first_raw + tick_distance) & 0xFFFFFFFF))
    return records


class ImuPublisherTests(unittest.TestCase):
    def make_publisher(self, consumer=None):
        consumer = consumer or RecordingConsumer()
        publisher = ImuPublisher(consumer)
        publisher.configure_clock(0)
        return publisher, consumer

    def test_publishes_complete_scaled_samples_at_reconstructed_times(self):
        publisher, consumer = self.make_publisher()

        publisher.process_notification(packet(0, anchored_records()))

        self.assertEqual(len(consumer.samples), 8)
        for index, sample in enumerate(consumer.samples):
            self.assertAlmostEqual(sample.timestamp_seconds, index / 960.0)

        first = consumer.samples[0]
        self.assertAlmostEqual(first.low_g.x, LOW_G_SCALE)
        self.assertAlmostEqual(first.low_g.y, -LOW_G_SCALE)
        self.assertAlmostEqual(first.high_g.z, 2 * HIGH_G_SCALE)
        self.assertAlmostEqual(first.gyro.x, GYRO_SCALE)

    def test_clock_diagnostic_uses_freq_fine(self):
        consumer = RecordingConsumer()
        publisher = ImuPublisher(consumer)

        publisher.configure_clock(-6)

        clock = next(
            item for item in consumer.diagnostics
            if isinstance(item, ClockDiagnostic)
        )
        correction = 1.0 + 0.0013 * -6
        self.assertAlmostEqual(clock.calculated_odr_hz, 960.0 * correction)
        self.assertAlmostEqual(
            clock.timestamp_tick_seconds,
            1.0 / (46080.0 * correction),
        )

    def test_assembles_slots_across_packet_boundaries(self):
        publisher, consumer = self.make_publisher()
        records = anchored_records()

        publisher.process_notification(packet(0, records[:7]))
        self.assertEqual(consumer.samples, [])
        publisher.process_notification(packet(1, records[7:]))

        self.assertEqual(len(consumer.samples), 8)
        packets = [
            item for item in consumer.diagnostics
            if isinstance(item, PacketDiagnostic)
        ]
        self.assertEqual([item.sequence for item in packets], [0, 1])

    def test_timestamp_and_tag_counter_rollover(self):
        publisher, consumer = self.make_publisher()
        first_raw = 0xFFFFFF00

        publisher.process_notification(
            packet(0xFFFFFFFF, anchored_records(first_raw=first_raw))
        )
        publisher.process_notification(packet(0, []))

        self.assertEqual(len(consumer.samples), 8)
        self.assertAlmostEqual(consumer.samples[-1].timestamp_seconds, 7 / 960.0)
        packet_diagnostics = [
            item for item in consumer.diagnostics
            if isinstance(item, PacketDiagnostic)
        ]
        self.assertEqual(packet_diagnostics[-1].missing_packets, 0)

    def test_incomplete_and_duplicate_slots_are_not_published(self):
        publisher, consumer = self.make_publisher()
        records = [timestamp_record(0, 1000)]

        for index in range(8):
            slot_records = complete_slot(index)
            if index == 2:
                slot_records = slot_records[:2]
            if index == 5:
                slot_records.append(slot_records[0])
            records.extend(slot_records)

        records.append(timestamp_record(0, 1384))
        publisher.process_notification(packet(0, records))

        self.assertEqual(len(consumer.samples), 6)
        discarded = [
            item for item in consumer.diagnostics
            if isinstance(item, TimingDiagnostic)
            and item.message == "incomplete FIFO time slot discarded"
        ]
        self.assertEqual(len(discarded), 2)

    def test_samples_before_first_anchor_are_discarded(self):
        publisher, consumer = self.make_publisher()
        records = complete_slot(0) + complete_slot(1)
        records.append(timestamp_record(2, 1000))
        for index in range(2, 10):
            records.extend(complete_slot(index))
        records.append(timestamp_record(2, 1384))

        publisher.process_notification(packet(0, records))

        self.assertEqual(len(consumer.samples), 8)
        self.assertAlmostEqual(consumer.samples[0].timestamp_seconds, 0.0)
        anchor = next(
            item for item in consumer.diagnostics
            if isinstance(item, TimingDiagnostic)
            and item.message == "timestamp reconstruction anchored"
        )
        self.assertEqual(anchor.discarded_samples, 2)

    def test_inconsistent_tag_counter_resets_reconstruction(self):
        publisher, consumer = self.make_publisher()
        records = [timestamp_record(0, 1000)]
        records.extend(complete_slot(0))
        records.extend(complete_slot(2))

        publisher.process_notification(packet(0, records))

        self.assertEqual(consumer.samples, [])
        resets = [
            item for item in consumer.diagnostics
            if isinstance(item, TimingDiagnostic)
            and item.message == "inconsistent FIFO time-slot progression"
        ]
        self.assertEqual(len(resets), 1)
        self.assertEqual(resets[0].discarded_samples, 1)

    def test_missing_packet_discards_uncertain_interval_and_reanchors(self):
        publisher, consumer = self.make_publisher()
        before_gap = [timestamp_record(0, 1000)]
        for index in range(4):
            before_gap.extend(complete_slot(index))
        publisher.process_notification(packet(0, before_gap))

        after_gap = anchored_records(first_raw=1384)
        publisher.process_notification(packet(2, after_gap))

        self.assertEqual(len(consumer.samples), 8)
        self.assertAlmostEqual(consumer.samples[0].timestamp_seconds, 1 / 120.0)
        packets = [
            item for item in consumer.diagnostics
            if isinstance(item, PacketDiagnostic)
        ]
        self.assertEqual(packets[-1].missing_packets, 1)
        resets = [
            item for item in consumer.diagnostics
            if isinstance(item, TimingDiagnostic)
            and item.message == "missing 1 BLE packet"
        ]
        self.assertEqual(len(resets), 1)
        self.assertEqual(resets[0].discarded_samples, 4)

    def test_malformed_packet_resets_without_partial_processing(self):
        publisher, consumer = self.make_publisher()
        publisher.process_notification(packet(0, [timestamp_record(0, 1000)]))

        malformed = struct.pack("<I", 1) + b"not-a-record"
        publisher.process_notification(malformed)
        publisher.process_notification(
            packet(2, anchored_records(first_raw=1384))
        )

        self.assertEqual(len(consumer.samples), 8)
        bad_packets = [
            item for item in consumer.diagnostics
            if isinstance(item, PacketDiagnostic) and not item.framing_valid
        ]
        self.assertEqual(len(bad_packets), 1)

    def test_duplicate_packet_is_ignored_and_reconstruction_resets(self):
        publisher, consumer = self.make_publisher()
        publisher.process_notification(packet(0, anchored_records()))
        self.assertEqual(len(consumer.samples), 8)

        publisher.process_notification(packet(0, anchored_records()))

        self.assertEqual(len(consumer.samples), 8)
        rejected = [
            item for item in consumer.diagnostics
            if isinstance(item, PacketDiagnostic) and not item.accepted
        ]
        self.assertEqual(len(rejected), 1)

    def test_minimal_consumer_only_implements_on_sample(self):
        consumer = MinimalConsumer()
        publisher, _ = self.make_publisher(consumer)

        publisher.process_notification(packet(0, anchored_records()))

        self.assertEqual(len(consumer.samples), 8)

    def test_close_discards_unanchored_trailing_slots(self):
        publisher, consumer = self.make_publisher()
        records = [timestamp_record(0, 1000)]
        records.extend(complete_slot(0))
        publisher.process_notification(packet(0, records))

        publisher.close()

        self.assertEqual(consumer.samples, [])
        close_diagnostics = [
            item for item in consumer.diagnostics
            if isinstance(item, TimingDiagnostic)
            and item.message == "stream ended before a closing anchor"
        ]
        self.assertEqual(close_diagnostics[0].discarded_samples, 1)


if __name__ == "__main__":
    unittest.main()
