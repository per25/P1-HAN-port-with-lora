# P1-HAN-port-with-lora

## Background

## About the project

Heltec ESP32 LoRa is a microcontroller board based on the ESP32, with built-in LoRa and 0.96 inch OLED Display. It is a perfect solution for IoT low power consumption and long range.

## How to run the project, nessacy installations

## Components of the project

### Hardware components
- Heltec ESP32 LoRa
- USB to Micro USB cable
- breadboard
- RJ12 cable(To the swedish smart meter)

### Software components
- Espressif ESP32 Development Framework: install ESP-IDF through here, [ESP-IDF for ESP32](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/index.html)
- ESP-IDF Extension in Visual Studio Code
- ESP-IDF has many diffrent project examples, one of them is the `UART_RX_TXtask` example. Which uses freertos to create a task that reads from the UART and writes to the UART. This is a good starting point for the project.

- LoraWan: Device library for ESP-IDF with SX127X Lora chips: [ttn-esp32](https://github.com/manuelbl/ttn-esp32)

- Drivers: to connect device to computer, [Link](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads)
