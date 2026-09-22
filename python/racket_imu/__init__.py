"""Public API for receiving complete racket IMU samples.

Typical use::

    class Consumer(ImuConsumer):
        def on_sample(self, sample: ImuSample) -> None:
            print(sample.timestamp_seconds, sample.low_g, sample.high_g, sample.gyro)

    asyncio.run(run(Consumer()))
"""

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


async def run(consumer: ImuConsumer) -> None:
    """Connect to the racket sensor and publish events to ``consumer``."""

    # Keep the optional Bleak dependency out of event-only consumers and tests.
    from .ble import run as run_ble_stream

    await run_ble_stream(consumer)


__all__ = [
    "ClockDiagnostic",
    "ConnectionDiagnostic",
    "ConnectionState",
    "Diagnostic",
    "ImuConsumer",
    "ImuSample",
    "PacketDiagnostic",
    "TimingDiagnostic",
    "Vector3",
    "run",
]
