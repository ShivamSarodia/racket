from io import StringIO
import unittest

from racket_imu import (
    ClockDiagnostic,
    ImuSample,
    PacketDiagnostic,
    TimingDiagnostic,
    Vector3,
)
from racket_imu.terminal import TerminalConsumer


class TerminalConsumerTests(unittest.TestCase):
    def test_dashboard_is_driven_only_by_consumer_callbacks(self):
        output = StringIO()
        consumer = TerminalConsumer(output)

        consumer.on_diagnostic(
            ClockDiagnostic(
                freq_fine=-6,
                timestamp_tick_seconds=0.00002,
                calculated_odr_hz=952.0,
            )
        )
        consumer.on_diagnostic(
            PacketDiagnostic(
                received_at=10.0,
                sequence=4,
                framing_valid=True,
                accepted=True,
                total_records=4,
                gyro_records=1,
                low_g_records=1,
                high_g_records=1,
                timestamp_records=1,
            )
        )
        consumer.on_diagnostic(
            TimingDiagnostic(
                message="timestamp interval reconstructed",
                anchors_seen=2,
                imu_elapsed_seconds=0.0084,
                anchor_spacing_seconds=0.0084,
                sample_period_seconds=0.00105,
            )
        )
        consumer.on_sample(
            ImuSample(
                timestamp_seconds=0.00735,
                low_g=Vector3(1.0, 2.0, 3.0),
                high_g=Vector3(4.0, 5.0, 6.0),
                gyro=Vector3(7.0, 8.0, 9.0),
            )
        )

        consumer.render(now=11.0)
        rendered = output.getvalue()

        self.assertIn("Low-g [g]          1.000     2.000     3.000", rendered)
        self.assertIn("High-g [g]         4.000     5.000     6.000", rendered)
        self.assertIn("Gyro [dps]           7.0       8.0       9.0", rendered)
        self.assertIn("Packets received:  1", rendered)
        self.assertIn("complete samples:  1", rendered)
        self.assertIn("FREQ_FINE:         -6", rendered)
        self.assertIn("Sample spacing:    1.050 ms", rendered)


if __name__ == "__main__":
    unittest.main()
