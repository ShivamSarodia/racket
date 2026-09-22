"""BLE transport for the racket IMU event publisher."""

from __future__ import annotations

import asyncio
import struct

from bleak import BleakClient, BleakScanner

from .events import ConnectionDiagnostic, ConnectionState, ImuConsumer
from .publisher import ImuPublisher


DEVICE_NAME = "RacketSensor"
DATA_UUID = "3a5f0002-9d7b-4c1b-a234-9ac1db720001"
CONTROL_UUID = "3a5f0003-9d7b-4c1b-a234-9ac1db720001"


async def run(consumer: ImuConsumer) -> None:
    """Connect, stream until cancelled, and deliver events to a consumer."""

    publisher = ImuPublisher(consumer)
    consumer.on_diagnostic(
        ConnectionDiagnostic(ConnectionState.SCANNING, DEVICE_NAME)
    )

    device = await BleakScanner.find_device_by_name(
        DEVICE_NAME,
        timeout=15.0,
    )

    if device is None:
        consumer.on_diagnostic(
            ConnectionDiagnostic(
                ConnectionState.DISCONNECTED,
                "Could not find BLE device {}".format(DEVICE_NAME),
            )
        )
        raise RuntimeError("Could not find BLE device {}".format(DEVICE_NAME))

    consumer.on_diagnostic(
        ConnectionDiagnostic(ConnectionState.CONNECTING, device.address)
    )

    try:
        async with BleakClient(device) as client:
            consumer.on_diagnostic(
                ConnectionDiagnostic(ConnectionState.CONNECTED, device.address)
            )

            freq_fine_raw = await client.read_gatt_char(CONTROL_UUID)

            if len(freq_fine_raw) != 1:
                raise RuntimeError(
                    "CONTROL characteristic did not return exactly "
                    "one FREQ_FINE byte"
                )

            freq_fine = struct.unpack("<b", bytes(freq_fine_raw))[0]
            publisher.configure_clock(freq_fine)

            def notification_handler(sender, data) -> None:
                del sender
                publisher.process_notification(bytes(data))

            notifying = False
            streaming = False

            try:
                await client.start_notify(DATA_UUID, notification_handler)
                notifying = True

                await client.write_gatt_char(
                    CONTROL_UUID,
                    b"\x01",
                    response=True,
                )
                streaming = True
                consumer.on_diagnostic(
                    ConnectionDiagnostic(ConnectionState.STREAMING)
                )

                await asyncio.Event().wait()

            finally:
                if streaming:
                    try:
                        await client.write_gatt_char(
                            CONTROL_UUID,
                            b"\x00",
                            response=True,
                        )
                    except Exception:
                        pass

                if notifying:
                    try:
                        await client.stop_notify(DATA_UUID)
                    except Exception:
                        pass

    finally:
        publisher.close()
        consumer.on_diagnostic(
            ConnectionDiagnostic(ConnectionState.DISCONNECTED)
        )
