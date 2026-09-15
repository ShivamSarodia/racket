arduino-cli compile --fqbn esp32:esp32:adafruit_feather_esp32_v2 .
PORT=$(ls /dev/cu.usbserial-* | head -1)
arduino-cli upload -p "$PORT" --fqbn esp32:esp32:adafruit_feather_esp32_v2 --board-options UploadSpeed=115200
