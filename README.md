# USB-To-BLE-Dongle
Makes your keyboard/mouse wireless
## How to:
- Clone this repository to your disk and run building using ESP-IDF extension for VSCode
## Todo:
- Keyboard LEDs support - done
- Keyboard consumer events support - done
- Measure the voltage on the Lion battery - done
- Solve high power consumption problems - maybe done
# Important:
- This code was written from ESP-IDF examples, also you need ESP32 board which supports USB protocol in hardware(in my case it is ESP32-S3-Zero and ESP-IDF v5.2)
- I used a voltage divider to measure the battery voltage, the first resistor is 100kOhm, the second is 300kOhm
