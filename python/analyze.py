"""Run the live racket analysis program."""

import asyncio
from contextlib import suppress

from math import sqrt
from racket_imu import run
from racket_imu.analysis import AnalysisConsumer


def max_tip_velocity(data):
    """
    Given collected data, return the maximum angular velocity.
    """
    return max(
        sqrt(d.gyro.x ** 2 + d.gyro.y ** 2 + d.gyro.z ** 2) for d in data
    )

async def main() -> None:
    consumer = AnalysisConsumer()
    asyncio.create_task(run(consumer))

    print("Starting recording in 3 seconds")
    await asyncio.sleep(3)
    print("Starting recording")
    consumer.start()
    await asyncio.sleep(3)
    data = consumer.end()

    print("Data collected:", len(data))
    print("Max tip velocity:", max_tip_velocity(data))


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print()
        print("Stopped.")
