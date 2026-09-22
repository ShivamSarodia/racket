"""Run the live racket IMU terminal dashboard."""

import asyncio
from contextlib import suppress

from racket_imu import run
from racket_imu.terminal import TerminalConsumer


async def main() -> None:
    consumer = TerminalConsumer()
    render_task = asyncio.create_task(consumer.render_forever())

    try:
        await run(consumer)
    finally:
        render_task.cancel()
        with suppress(asyncio.CancelledError):
            await render_task


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print()
        print("Stopped.")
