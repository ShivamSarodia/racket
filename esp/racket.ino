#include <Arduino.h>
#include <SPI.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <esp_gap_ble_api.h>
#include <esp_gatt_common_api.h>
#include <esp_err.h>

#if !defined(CONFIG_BLUEDROID_ENABLED)
#error "This sketch requires the ESP32 Bluedroid BLE stack."
#endif

constexpr int CS_PIN   = 27;
constexpr int SCK_PIN  = 5;
constexpr int MISO_PIN = 21;
constexpr int MOSI_PIN = 19;

SPISettings imuSPI(5000000, MSBFIRST, SPI_MODE3);

constexpr char DEVICE_NAME[] = "RacketSensor";
constexpr char SERVICE_UUID[] = "3a5f0001-9d7b-4c1b-a234-9ac1db720001";
constexpr char DATA_UUID[] = "3a5f0002-9d7b-4c1b-a234-9ac1db720001";
constexpr char CONTROL_UUID[] = "3a5f0003-9d7b-4c1b-a234-9ac1db720001";

BLEServer *bleServer = nullptr;
BLECharacteristic *dataCharacteristic = nullptr;
BLECharacteristic *controlCharacteristic = nullptr;

volatile bool deviceConnected = false;
volatile bool startStreamingRequested = false;
volatile bool stopStreamingRequested = false;
bool streamingEnabled = false;

constexpr uint16_t LOCAL_MTU = 185;
constexpr size_t MAX_NOTIFY_BYTES = LOCAL_MTU - 3;
constexpr uint16_t REQUESTED_DATA_LENGTH = 251;
constexpr uint16_t CONN_INTERVAL_MIN = 12; // 15 ms
constexpr uint16_t CONN_INTERVAL_MAX = 12;
constexpr uint16_t CONN_LATENCY = 0;
constexpr uint16_t CONN_TIMEOUT = 400; // 4 s

esp_bd_addr_t peerAddress;
volatile uint16_t currentConnId = 0;
volatile bool linkTuningPending = false;
uint32_t connectedAtMs = 0;

esp_err_t lastDleRequestResult = ESP_OK;
bool lastConnParamRequestResult = false;
volatile uint16_t negotiatedIntervalUnits = 0;
volatile uint16_t negotiatedLatency = 0;
volatile uint16_t negotiatedTimeoutUnits = 0;
volatile int negotiatedParamStatus = -1;

// Minimal BLE packet:
//   bytes 0..3   uint32 packet sequence, little-endian
//   bytes 4..end raw 7-byte IMU FIFO records
constexpr size_t PACKET_HEADER_SIZE = 4;
constexpr size_t FIFO_RECORD_SIZE = 7;
uint32_t packetSequence = 0;

// USB-only diagnostics.
uint32_t packetsSubmitted = 0;
volatile uint32_t notifySuccess = 0;
volatile uint32_t notifyGattErrors = 0;
volatile uint32_t notifyDisabledErrors = 0;
volatile uint32_t notifyNoClientErrors = 0;
volatile uint32_t notifyNoSubscriberErrors = 0;
volatile uint32_t notifyOtherErrors = 0;
volatile uint32_t lastNotifyStatusCode = 0;
uint32_t sendSlotChecks = 0;
uint32_t zeroSendSlotChecks = 0;
uint16_t lastSendableSlots = 0;
uint16_t maxSendableSlotsSeen = 0;

// LSM6DSV320X registers.
constexpr uint8_t REG_FIFO_CTRL1         = 0x07;
constexpr uint8_t REG_FIFO_CTRL2         = 0x08;
constexpr uint8_t REG_FIFO_CTRL3         = 0x09;
constexpr uint8_t REG_FIFO_CTRL4         = 0x0A;
constexpr uint8_t REG_COUNTER_BDR1       = 0x0B;
constexpr uint8_t REG_WHO_AM_I           = 0x0F;
constexpr uint8_t REG_CTRL1              = 0x10;
constexpr uint8_t REG_CTRL2              = 0x11;
constexpr uint8_t REG_CTRL3              = 0x12;
constexpr uint8_t REG_CTRL6              = 0x15;
constexpr uint8_t REG_CTRL8              = 0x17;
constexpr uint8_t REG_FIFO_STATUS1       = 0x1B;
constexpr uint8_t REG_CTRL1_XL_HG        = 0x4E;
constexpr uint8_t REG_INTERNAL_FREQ_FINE = 0x4F;
constexpr uint8_t REG_FUNCTIONS_ENABLE   = 0x50;
constexpr uint8_t REG_FIFO_DATA          = 0x78;

constexpr uint8_t FIFO_TAG_GYRO      = 0x01;
constexpr uint8_t FIFO_TAG_LOW_G     = 0x02;
constexpr uint8_t FIFO_TAG_TIMESTAMP = 0x04;
constexpr uint8_t FIFO_TAG_HIGH_G    = 0x1D;

int8_t imuFreqFine = 0;
double imuTimestampTickSeconds = 0.0;
double imuActualOdrHz = 0.0;
uint32_t timestampRecordsSeen = 0;

struct RawFifoRecord {
  uint8_t bytes[FIFO_RECORD_SIZE];
};

constexpr size_t PSRAM_RING_CAPACITY = 65536;
constexpr size_t DRAM_RING_CAPACITY  = 8192;

RawFifoRecord *ringBuffer = nullptr;
size_t ringCapacity = 0;
size_t ringReadIndex = 0;
size_t ringWriteIndex = 0;
size_t ringCount = 0;
uint32_t ringDroppedRecords = 0;
bool fifoOverrunSeen = false;

uint8_t readReg(uint8_t reg) {
  SPI.beginTransaction(imuSPI);
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg | 0x80);
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  return value;
}

void writeReg(uint8_t reg, uint8_t value) {
  SPI.beginTransaction(imuSPI);
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(value);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
}

void readRegs(uint8_t startReg, uint8_t *buffer, size_t length) {
  SPI.beginTransaction(imuSPI);
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(startReg | 0x80);
  for (size_t i = 0; i < length; i++) {
    buffer[i] = SPI.transfer(0x00);
  }
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
}

void resetRingBuffer() {
  ringReadIndex = 0;
  ringWriteIndex = 0;
  ringCount = 0;
  ringDroppedRecords = 0;
}

void pushRingRecord(const uint8_t *record) {
  if (ringCount == ringCapacity) {
    ringReadIndex = (ringReadIndex + 1) % ringCapacity;
    ringCount--;
    ringDroppedRecords++;
  }

  memcpy(ringBuffer[ringWriteIndex].bytes, record, FIFO_RECORD_SIZE);
  ringWriteIndex = (ringWriteIndex + 1) % ringCapacity;
  ringCount++;
}

void popRingRecords(size_t count) {
  if (count > ringCount) count = ringCount;
  ringReadIndex = (ringReadIndex + count) % ringCapacity;
  ringCount -= count;
}

uint16_t getFifoLevel() {
  uint8_t status[2];
  readRegs(REG_FIFO_STATUS1, status, 2);

  uint16_t level =
      (uint16_t)status[0]
      |
      (((uint16_t)(status[1] & 0x01)) << 8);

  if (status[1] & 0x40) {
    fifoOverrunSeen = true;
  }

  return level;
}

void drainImuFifoToRam() {
  while (true) {
    uint16_t level = getFifoLevel();
    if (level == 0) return;

    for (uint16_t i = 0; i < level; i++) {
      uint8_t raw[FIFO_RECORD_SIZE];
      readRegs(REG_FIFO_DATA, raw, FIFO_RECORD_SIZE);

      if ((raw[0] >> 3) == FIFO_TAG_TIMESTAMP) {
        timestampRecordsSeen++;
      }

      pushRingRecord(raw);
    }
  }
}

void discardImuFifo() {
  while (true) {
    uint16_t level = getFifoLevel();
    if (level == 0) return;

    for (uint16_t i = 0; i < level; i++) {
      uint8_t raw[FIFO_RECORD_SIZE];
      readRegs(REG_FIFO_DATA, raw, FIFO_RECORD_SIZE);
    }
  }
}

void configureSensor() {
  uint8_t who = readReg(REG_WHO_AM_I);
  Serial.print("WHO_AM_I = 0x");
  Serial.println(who, HEX);

  if (who != 0x73) {
    Serial.println("ERROR: LSM6DSV320X not detected.");
    while (true) delay(1000);
  }

  // FIFO bypass while configuring.
  writeReg(REG_FIFO_CTRL4, 0x00);

  // BDU + auto-increment.
  uint8_t ctrl3 = readReg(REG_CTRL3);
  ctrl3 |= 0x44;
  writeReg(REG_CTRL3, ctrl3);

  // Low-g +/-16 g.
  uint8_t ctrl8 = readReg(REG_CTRL8);
  ctrl8 &= 0xFC;
  ctrl8 |= 0x03;
  writeReg(REG_CTRL8, ctrl8);

  // Gyro +/-4000 dps.
  writeReg(REG_CTRL6, 0x0D);

  // High-g output enabled, nominal 960 Hz, +/-320 g.
  writeReg(REG_CTRL1_XL_HG, 0xA4);

  // Low-g nominal 960 Hz.
  uint8_t ctrl1 = readReg(REG_CTRL1);
  ctrl1 &= 0xF0;
  ctrl1 |= 0x09;
  writeReg(REG_CTRL1, ctrl1);

  // Gyro nominal 960 Hz.
  uint8_t ctrl2 = readReg(REG_CTRL2);
  ctrl2 &= 0xF0;
  ctrl2 |= 0x09;
  writeReg(REG_CTRL2, ctrl2);

  // Low-g + gyro FIFO batching at nominal 960 Hz.
  writeReg(REG_FIFO_CTRL3, 0x99);

  // High-g FIFO batching.
  uint8_t counterBdr = readReg(REG_COUNTER_BDR1);
  counterBdr |= 0x08;
  writeReg(REG_COUNTER_BDR1, counterBdr);

  // Enable timestamp counter: FUNCTIONS_ENABLE bit 6.
  uint8_t functionsEnable = readReg(REG_FUNCTIONS_ENABLE);
  functionsEnable |= 0x40;
  writeReg(REG_FUNCTIONS_ENABLE, functionsEnable);

  // Read signed per-device frequency calibration.
  imuFreqFine = (int8_t)readReg(REG_INTERNAL_FREQ_FINE);

  double clockCorrection = 1.0 + 0.0013 * (double)imuFreqFine;

  imuTimestampTickSeconds =
      1.0 / (46080.0 * clockCorrection);

  // Selected ODR = 960 Hz, ODR_coeff = 8.
  imuActualOdrHz = 960.0 * clockCorrection;

  // No watermark / compression.
  writeReg(REG_FIFO_CTRL1, 0x00);
  writeReg(REG_FIFO_CTRL2, 0x00);

  // FIFO_CTRL4:
  //   bits 2:0 = 110 = stream mode
  //   bits 7:6 = 10  = timestamp DEC_8
  // => 0x86
  writeReg(REG_FIFO_CTRL4, 0x86);

  delay(100);

  Serial.println();
  Serial.println("IMU configured:");
  Serial.println("  Low-g:     +/-16 g @ nominal 960 Hz");
  Serial.println("  High-g:    +/-320 g @ nominal 960 Hz");
  Serial.println("  Gyro:      +/-4000 dps @ nominal 960 Hz");
  Serial.println("  FIFO:      stream mode");
  Serial.println("  Timestamp: enabled, FIFO DEC_8");
  Serial.print("  FREQ_FINE: ");
  Serial.println((int)imuFreqFine);
  Serial.print("  Timestamp tick: ");
  Serial.print(imuTimestampTickSeconds * 1000000.0, 4);
  Serial.println(" us");
  Serial.print("  Calculated actual ODR: ");
  Serial.print(imuActualOdrHz, 3);
  Serial.println(" Hz");
  Serial.print("  Expected timestamp FIFO rate: ");
  Serial.print(imuActualOdrHz / 8.0, 3);
  Serial.println(" Hz");
  Serial.println();
}

class DataCallbacks : public BLECharacteristicCallbacks {
  void onStatus(
      BLECharacteristic *characteristic,
      Status status,
      uint32_t code
  ) override {
    switch (status) {
      case SUCCESS_NOTIFY:
        notifySuccess++;
        break;
      case ERROR_GATT:
        notifyGattErrors++;
        lastNotifyStatusCode = code;
        break;
      case ERROR_NOTIFY_DISABLED:
        notifyDisabledErrors++;
        lastNotifyStatusCode = code;
        break;
      case ERROR_NO_CLIENT:
        notifyNoClientErrors++;
        lastNotifyStatusCode = code;
        break;
      case ERROR_NO_SUBSCRIBER:
        notifyNoSubscriberErrors++;
        lastNotifyStatusCode = code;
        break;
      default:
        notifyOtherErrors++;
        lastNotifyStatusCode = code;
        break;
    }
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(
      BLEServer *server,
      esp_ble_gatts_cb_param_t *param
  ) override {
    deviceConnected = true;
    currentConnId = param->connect.conn_id;

    memcpy(
        peerAddress,
        param->connect.remote_bda,
        sizeof(esp_bd_addr_t)
    );

    connectedAtMs = millis();
    linkTuningPending = true;

    Serial.print("BLE client connected. conn_id=");
    Serial.println(currentConnId);
  }

  void onDisconnect(
      BLEServer *server,
      esp_ble_gatts_cb_param_t *param
  ) override {
    deviceConnected = false;
    streamingEnabled = false;
    stopStreamingRequested = false;
    startStreamingRequested = false;
    linkTuningPending = false;

    Serial.println("BLE client disconnected.");
  }

  void onConnParamsUpdate(
      esp_bd_addr_t remote_bda,
      uint16_t interval,
      uint16_t latency,
      uint16_t timeout,
      esp_bt_status_t status
  ) override {
    negotiatedIntervalUnits = interval;
    negotiatedLatency = latency;
    negotiatedTimeoutUnits = timeout;
    negotiatedParamStatus = (int)status;

    Serial.print("BLE connection params updated: interval=");
    Serial.print(interval * 1.25f, 2);
    Serial.print(" ms latency=");
    Serial.print(latency);
    Serial.print(" timeout=");
    Serial.print(timeout * 10);
    Serial.print(" ms status=");
    Serial.println((int)status);
  }
};

// CONTROL characteristic:
//   READ  -> one signed byte: FREQ_FINE
//   WRITE -> 0x01 start, 0x00 stop
class ControlCallbacks : public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *characteristic) override {
    uint8_t rawFreqFine = (uint8_t)imuFreqFine;
    characteristic->setValue(&rawFreqFine, 1);
  }

  void onWrite(BLECharacteristic *characteristic) override {
    String value = characteristic->getValue();
    if (value.length() < 1) return;

    uint8_t command = (uint8_t)value[0];

    if (command == 0x01) {
      startStreamingRequested = true;
    } else if (command == 0x00) {
      stopStreamingRequested = true;
    }
  }
};

void setupBle() {
  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU(LOCAL_MTU);

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());
  bleServer->advertiseOnDisconnect(true);

  BLEService *service = bleServer->createService(SERVICE_UUID);

  dataCharacteristic = service->createCharacteristic(
      DATA_UUID,
      BLECharacteristic::PROPERTY_NOTIFY
  );
  dataCharacteristic->addDescriptor(new BLE2902());
  dataCharacteristic->setCallbacks(new DataCallbacks());

  controlCharacteristic = service->createCharacteristic(
      CONTROL_UUID,
      BLECharacteristic::PROPERTY_READ
      |
      BLECharacteristic::PROPERTY_WRITE
  );
  controlCharacteristic->setCallbacks(new ControlCallbacks());

  uint8_t rawFreqFine = (uint8_t)imuFreqFine;
  controlCharacteristic->setValue(&rawFreqFine, 1);

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(CONN_INTERVAL_MIN);
  advertising->setMaxPreferred(CONN_INTERVAL_MAX);

  BLEDevice::startAdvertising();

  Serial.println("BLE advertising as RacketSensor");
}

void handleLinkTuning() {
  if (!deviceConnected || !linkTuningPending) return;
  if (millis() - connectedAtMs < 100) return;

  linkTuningPending = false;

  lastDleRequestResult = esp_ble_gap_set_pkt_data_len(
      peerAddress,
      REQUESTED_DATA_LENGTH
  );

  Serial.print("DLE request (251 bytes): ");
  Serial.println(esp_err_to_name(lastDleRequestResult));

  lastConnParamRequestResult = bleServer->requestConnParams(
      peerAddress,
      CONN_INTERVAL_MIN,
      CONN_INTERVAL_MAX,
      CONN_LATENCY,
      CONN_TIMEOUT
  );

  Serial.print("15 ms connection interval request: ");
  Serial.println(
      lastConnParamRequestResult
          ? "accepted for negotiation"
          : "request failed"
  );
}

void resetNotificationDiagnostics() {
  packetsSubmitted = 0;
  notifySuccess = 0;
  notifyGattErrors = 0;
  notifyDisabledErrors = 0;
  notifyNoClientErrors = 0;
  notifyNoSubscriberErrors = 0;
  notifyOtherErrors = 0;
  lastNotifyStatusCode = 0;
  sendSlotChecks = 0;
  zeroSendSlotChecks = 0;
  lastSendableSlots = 0;
  maxSendableSlotsSeen = 0;
  timestampRecordsSeen = 0;
}

void handleControlRequests() {
  if (stopStreamingRequested) {
    stopStreamingRequested = false;
    streamingEnabled = false;
    resetRingBuffer();
    Serial.println("BLE streaming stopped.");
  }

  if (startStreamingRequested) {
    startStreamingRequested = false;

    discardImuFifo();
    resetRingBuffer();
    fifoOverrunSeen = false;
    packetSequence = 0;
    resetNotificationDiagnostics();
    streamingEnabled = true;

    Serial.println("BLE streaming started.");
  }
}

void writeU32LE(uint8_t *dst, uint32_t value) {
  dst[0] = value & 0xFF;
  dst[1] = (value >> 8) & 0xFF;
  dst[2] = (value >> 16) & 0xFF;
  dst[3] = (value >> 24) & 0xFF;
}

size_t maxRecordsPerBlePacket() {
  if (!deviceConnected) return 0;

  uint16_t peerMtu = bleServer->getPeerMTU(currentConnId);

  size_t maxPayload = 20;
  if (peerMtu > 3) maxPayload = peerMtu - 3;
  if (maxPayload > MAX_NOTIFY_BYTES) maxPayload = MAX_NOTIFY_BYTES;
  if (maxPayload <= PACKET_HEADER_SIZE) return 0;

  return
      (maxPayload - PACKET_HEADER_SIZE)
      / FIFO_RECORD_SIZE;
}

void submitOnePacket(size_t recordsPerPacket) {
  uint8_t packet[MAX_NOTIFY_BYTES];

  // Only BLE-level metadata: packet sequence.
  writeU32LE(&packet[0], packetSequence);

  size_t outputOffset = PACKET_HEADER_SIZE;
  size_t index = ringReadIndex;

  for (size_t i = 0; i < recordsPerPacket; i++) {
    memcpy(
        &packet[outputOffset],
        ringBuffer[index].bytes,
        FIFO_RECORD_SIZE
    );

    outputOffset += FIFO_RECORD_SIZE;
    index = (index + 1) % ringCapacity;
  }

  packetsSubmitted++;

  dataCharacteristic->setValue(packet, outputOffset);
  dataCharacteristic->notify();

  packetSequence++;

  // No laptop ACK yet. Controller send-slot flow control prevents
  // over-submitting notifications to Bluedroid.
  popRingRecords(recordsPerPacket);
}

void sendAvailableBlePackets() {
  if (!deviceConnected || !streamingEnabled) return;

  size_t recordsPerPacket = maxRecordsPerBlePacket();

  if (
      recordsPerPacket == 0
      ||
      ringCount < recordsPerPacket
  ) {
    return;
  }

  uint16_t sendable = esp_ble_get_cur_sendable_packets_num(
      currentConnId
  );

  sendSlotChecks++;
  lastSendableSlots = sendable;

  if (sendable > maxSendableSlotsSeen) {
    maxSendableSlotsSeen = sendable;
  }

  if (sendable == 0) {
    zeroSendSlotChecks++;
    return;
  }

  while (
      sendable > 0
      &&
      ringCount >= recordsPerPacket
  ) {
    submitOnePacket(recordsPerPacket);
    sendable--;

    // Keep draining the IMU between notifications.
    drainImuFifoToRam();
  }
}

void allocateRingBuffer() {
  if (psramFound()) {
    ringCapacity = PSRAM_RING_CAPACITY;

    ringBuffer = (RawFifoRecord *)ps_malloc(
        ringCapacity * sizeof(RawFifoRecord)
    );

    if (ringBuffer != nullptr) {
      Serial.print("Ring buffer in PSRAM: ");
      Serial.print(ringCapacity);
      Serial.println(" records");
      return;
    }
  }

  ringCapacity = DRAM_RING_CAPACITY;

  ringBuffer = (RawFifoRecord *)malloc(
      ringCapacity * sizeof(RawFifoRecord)
  );

  if (ringBuffer == nullptr) {
    Serial.println("ERROR: could not allocate ring buffer.");
    while (true) delay(1000);
  }

  Serial.print("Ring buffer in DRAM: ");
  Serial.print(ringCapacity);
  Serial.println(" records");
}

void printDiagnostics() {
  static uint32_t lastPrintMs = 0;

  if (millis() - lastPrintMs < 2000) return;
  lastPrintMs = millis();

  Serial.print("BLE=");
  Serial.print(deviceConnected ? "connected" : "waiting");
  Serial.print(" stream=");
  Serial.print(streamingEnabled ? "on" : "off");
  Serial.print(" ring=");
  Serial.print(ringCount);
  Serial.print("/");
  Serial.print(ringCapacity);
  Serial.print(" dropped=");
  Serial.print(ringDroppedRecords);
  Serial.print(" fifo_overrun=");
  Serial.print(fifoOverrunSeen ? "YES" : "no");
  Serial.print(" timestamp_records=");
  Serial.println(timestampRecordsSeen);

  Serial.print("BLE TX: submitted=");
  Serial.print(packetsSubmitted);
  Serial.print(" success=");
  Serial.print(notifySuccess);
  Serial.print(" gatt_err=");
  Serial.print(notifyGattErrors);
  Serial.print(" other_err=");
  Serial.print(
      notifyOtherErrors
      + notifyDisabledErrors
      + notifyNoClientErrors
      + notifyNoSubscriberErrors
  );
  Serial.print(" last_code=");
  Serial.print((int32_t)lastNotifyStatusCode);
  Serial.print(" send_slots_now=");
  Serial.print(lastSendableSlots);
  Serial.print(" max_slots=");
  Serial.print(maxSendableSlotsSeen);
  Serial.print(" zero_slot_checks=");
  Serial.print(zeroSendSlotChecks);

  if (deviceConnected) {
    Serial.print(" mtu=");
    Serial.print(bleServer->getPeerMTU(currentConnId));
    Serial.print(" records_per_packet=");
    Serial.print(maxRecordsPerBlePacket());
  }

  Serial.println();

  if (negotiatedIntervalUnits > 0) {
    Serial.print("BLE LINK: interval=");
    Serial.print(negotiatedIntervalUnits * 1.25f, 2);
    Serial.print("ms latency=");
    Serial.print(negotiatedLatency);
    Serial.print(" timeout=");
    Serial.print(negotiatedTimeoutUnits * 10);
    Serial.print("ms status=");
    Serial.println(negotiatedParamStatus);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  delay(100);

  allocateRingBuffer();
  configureSensor();
  setupBle();

  Serial.println();
  Serial.println("Sensor running.");
  Serial.println("Idle IMU data discarded until laptop starts streaming.");
  Serial.println();
}

void loop() {
  handleControlRequests();
  handleLinkTuning();

  if (streamingEnabled) {
    drainImuFifoToRam();
  } else {
    discardImuFifo();
  }

  sendAvailableBlePackets();
  printDiagnostics();
}
