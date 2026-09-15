import asyncio
import struct
import time

from bleak import BleakClient, BleakScanner


DEVICE_NAME = "RacketSensor"

DATA_UUID = "3a5f0002-9d7b-4c1b-a234-9ac1db720001"
CONTROL_UUID = "3a5f0003-9d7b-4c1b-a234-9ac1db720001"

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

LOW_G_SCALE = 0.000488      # g / LSB
HIGH_G_SCALE = 0.010417     # g / LSB
GYRO_SCALE = 0.140          # dps / LSB


class State:
    def __init__(self):
        self.low_g = [0.0, 0.0, 0.0]
        self.high_g = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]

        self.low_g_records = 0
        self.high_g_records = 0
        self.gyro_records = 0
        self.timestamp_records = 0
        self.other_records = 0
        self.total_records = 0

        self.packets_received = 0
        self.missing_packets = 0
        self.bad_packets = 0
        self.expected_sequence = None

        self.first_packet_time = None
        self.last_packet_time = None

        # Read once from CONTROL at connection time.
        self.freq_fine = None
        self.timestamp_tick_seconds = None
        self.calculated_odr_hz = None

        # 32-bit hardware timestamp unwrapping.
        self.last_timestamp_raw = None
        self.timestamp_wraps = 0
        self.first_timestamp_unwrapped = None
        self.last_timestamp_unwrapped = None
        self.last_timestamp_spacing_seconds = None


state = State()


def magnitude(v):
    return (
        v[0] * v[0]
        + v[1] * v[1]
        + v[2] * v[2]
    ) ** 0.5


def configure_imu_clock(freq_fine):
    state.freq_fine = freq_fine

    correction = 1.0 + 0.0013 * freq_fine

    # ST formula:
    # t_actual = 1 / (46080 * (1 + 0.0013 * FREQ_FINE))
    state.timestamp_tick_seconds = (
        1.0
        /
        (46080.0 * correction)
    )

    # Selected ODR = 960 Hz, ODR_coeff = 8.
    state.calculated_odr_hz = 960.0 * correction


def process_timestamp(raw_timestamp):
    if state.last_timestamp_raw is not None:
        # Detect normal uint32 rollover, not a small backwards glitch.
        if (
            raw_timestamp < state.last_timestamp_raw
            and
            (
                state.last_timestamp_raw
                - raw_timestamp
            ) > 0x80000000
        ):
            state.timestamp_wraps += 1

    unwrapped = (
        raw_timestamp
        + state.timestamp_wraps * (1 << 32)
    )

    if state.first_timestamp_unwrapped is None:
        state.first_timestamp_unwrapped = unwrapped

    if (
        state.last_timestamp_unwrapped is not None
        and
        state.timestamp_tick_seconds is not None
    ):
        delta_ticks = (
            unwrapped
            - state.last_timestamp_unwrapped
        )

        state.last_timestamp_spacing_seconds = (
            delta_ticks
            * state.timestamp_tick_seconds
        )

    state.last_timestamp_raw = raw_timestamp
    state.last_timestamp_unwrapped = unwrapped


def process_fifo_record(record):
    sensor_tag = record[0] >> 3

    if sensor_tag == TAG_GYRO:
        x, y, z = struct.unpack_from("<hhh", record, 1)
        state.gyro = [
            x * GYRO_SCALE,
            y * GYRO_SCALE,
            z * GYRO_SCALE,
        ]
        state.gyro_records += 1

    elif sensor_tag == TAG_LOW_G:
        x, y, z = struct.unpack_from("<hhh", record, 1)
        state.low_g = [
            x * LOW_G_SCALE,
            y * LOW_G_SCALE,
            z * LOW_G_SCALE,
        ]
        state.low_g_records += 1

    elif sensor_tag == TAG_HIGH_G:
        x, y, z = struct.unpack_from("<hhh", record, 1)
        state.high_g = [
            x * HIGH_G_SCALE,
            y * HIGH_G_SCALE,
            z * HIGH_G_SCALE,
        ]
        state.high_g_records += 1

    elif sensor_tag == TAG_TIMESTAMP:
        # First four FIFO payload bytes are the uint32 timestamp.
        raw_timestamp = struct.unpack_from("<I", record, 1)[0]
        state.timestamp_records += 1
        process_timestamp(raw_timestamp)

    else:
        state.other_records += 1

    state.total_records += 1


def notification_handler(sender, data):
    now = time.monotonic()

    if len(data) < PACKET_HEADER_SIZE:
        state.bad_packets += 1
        return

    fifo_bytes = len(data) - PACKET_HEADER_SIZE

    # No record-count field: infer it from notification length.
    if fifo_bytes % FIFO_RECORD_SIZE != 0:
        state.bad_packets += 1
        return

    sequence = struct.unpack_from("<I", data, 0)[0]

    if state.expected_sequence is None:
        state.expected_sequence = (sequence + 1) & 0xFFFFFFFF

    else:
        if sequence != state.expected_sequence:
            gap = (
                sequence
                - state.expected_sequence
            ) & 0xFFFFFFFF

            # Only count a forward gap.
            if gap < 0x80000000:
                state.missing_packets += gap

        state.expected_sequence = (sequence + 1) & 0xFFFFFFFF

    state.packets_received += 1
    state.last_packet_time = now

    if state.first_packet_time is None:
        state.first_packet_time = now

    for offset in range(
        PACKET_HEADER_SIZE,
        len(data),
        FIFO_RECORD_SIZE,
    ):
        process_fifo_record(
            data[offset:offset + FIFO_RECORD_SIZE]
        )


def render():
    now = time.monotonic()

    if (
        state.first_packet_time is not None
        and
        now > state.first_packet_time
    ):
        elapsed = now - state.first_packet_time

        fifo_rate = state.total_records / elapsed
        gyro_rate = state.gyro_records / elapsed
        low_g_rate = state.low_g_records / elapsed
        high_g_rate = state.high_g_records / elapsed
        timestamp_rate = state.timestamp_records / elapsed

    else:
        fifo_rate = 0.0
        gyro_rate = 0.0
        low_g_rate = 0.0
        high_g_rate = 0.0
        timestamp_rate = 0.0

    if state.last_packet_time is None:
        packet_age_ms = None
    else:
        packet_age_ms = (
            now - state.last_packet_time
        ) * 1000.0

    imu_time_seconds = None

    if (
        state.first_timestamp_unwrapped is not None
        and
        state.last_timestamp_unwrapped is not None
        and
        state.timestamp_tick_seconds is not None
    ):
        imu_time_seconds = (
            state.last_timestamp_unwrapped
            - state.first_timestamp_unwrapped
        ) * state.timestamp_tick_seconds

    print("\033[H", end="")

    print("Racket IMU — Live")
    print("=================")
    print()

    print(
        f"{'':14}"
        f"{'X':>10}"
        f"{'Y':>10}"
        f"{'Z':>10}"
    )

    print(
        f"{'Low-g [g]':14}"
        f"{state.low_g[0]:10.3f}"
        f"{state.low_g[1]:10.3f}"
        f"{state.low_g[2]:10.3f}"
    )

    print(
        f"{'High-g [g]':14}"
        f"{state.high_g[0]:10.3f}"
        f"{state.high_g[1]:10.3f}"
        f"{state.high_g[2]:10.3f}"
    )

    print(
        f"{'Gyro [dps]':14}"
        f"{state.gyro[0]:10.1f}"
        f"{state.gyro[1]:10.1f}"
        f"{state.gyro[2]:10.1f}"
    )

    print()
    print(f"|low-g|       {magnitude(state.low_g):8.3f} g")
    print(f"|high-g|      {magnitude(state.high_g):8.3f} g")
    print(f"|gyro|        {magnitude(state.gyro):8.1f} dps")

    print()
    print("Transport")
    print("---------")
    print(f"Packets received:  {state.packets_received}")
    print(f"Missing packets:   {state.missing_packets}")
    print(f"Bad packets:       {state.bad_packets}")
    print(f"FIFO records/sec:  {fifo_rate:.1f}")

    print()
    print(
        f"gyro records:      {state.gyro_records}"
        f"   ({gyro_rate:.1f}/s)"
    )
    print(
        f"low-g records:     {state.low_g_records}"
        f"   ({low_g_rate:.1f}/s)"
    )
    print(
        f"high-g records:    {state.high_g_records}"
        f"   ({high_g_rate:.1f}/s)"
    )
    print(
        f"timestamp records: {state.timestamp_records}"
        f"   ({timestamp_rate:.1f}/s)"
    )

    print()
    print("IMU timing")
    print("----------")

    if state.freq_fine is None:
        print("FREQ_FINE:         unavailable")
    else:
        print(f"FREQ_FINE:         {state.freq_fine}")
        print(
            f"Timestamp tick:    "
            f"{state.timestamp_tick_seconds * 1e6:.4f} us"
        )
        print(
            f"Calculated ODR:    "
            f"{state.calculated_odr_hz:.3f} Hz"
        )
        print(
            f"Expected TS rate:  "
            f"{state.calculated_odr_hz / 8.0:.3f} Hz"
        )

    if imu_time_seconds is None:
        print("IMU elapsed time:  waiting for timestamps...")
    else:
        print(f"IMU elapsed time:  {imu_time_seconds:.6f} s")

    if state.last_timestamp_spacing_seconds is not None:
        print(
            f"Last TS spacing:   "
            f"{state.last_timestamp_spacing_seconds * 1000.0:.3f} ms"
        )

    print()

    if packet_age_ms is None:
        print("Last BLE packet:   waiting...")
    else:
        print(f"Last BLE packet:   {packet_age_ms:.1f} ms ago")

    print()
    print("Ctrl-C to quit.")


async def main():
    print(f"Scanning for {DEVICE_NAME}...")

    device = await BleakScanner.find_device_by_name(
        DEVICE_NAME,
        timeout=15.0,
    )

    if device is None:
        raise RuntimeError(
            f"Could not find BLE device {DEVICE_NAME}"
        )

    print(f"Connecting to {device.address}...")

    async with BleakClient(device) as client:
        print("Connected.")

        # Read FREQ_FINE once. CONTROL returns one signed byte.
        freq_fine_raw = await client.read_gatt_char(CONTROL_UUID)

        if len(freq_fine_raw) != 1:
            raise RuntimeError(
                "CONTROL characteristic did not return exactly "
                "one FREQ_FINE byte"
            )

        freq_fine = struct.unpack(
            "<b",
            bytes(freq_fine_raw),
        )[0]

        configure_imu_clock(freq_fine)

        print(
            f"FREQ_FINE={freq_fine}, "
            f"timestamp tick="
            f"{state.timestamp_tick_seconds * 1e6:.4f} us, "
            f"calculated ODR="
            f"{state.calculated_odr_hz:.3f} Hz"
        )

        # Subscribe first, then ask Feather to begin sending fresh data.
        await client.start_notify(
            DATA_UUID,
            notification_handler,
        )

        await client.write_gatt_char(
            CONTROL_UUID,
            b"\x01",
            response=True,
        )

        print("\033[2J\033[H", end="")

        try:
            while True:
                render()
                await asyncio.sleep(0.1)

        finally:
            try:
                await client.write_gatt_char(
                    CONTROL_UUID,
                    b"\x00",
                    response=True,
                )
            except Exception:
                pass

            try:
                await client.stop_notify(DATA_UUID)
            except Exception:
                pass


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print()
        print("Stopped.")
