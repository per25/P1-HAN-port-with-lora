# P1-HAN-port-with-lora

## Background

In recent years, the technology for reading data from home electrical meters 
has significantly evolved. A multitude of commercial products are now 
available, offering real-time data capture capabilities from these meters. 
These products range in complexity: some are basic, designed to transmit 
data to an online interface through Wi-Fi, while others are more 
sophisticated, using meter data to optimize tasks like load balancing for 
electric car chargers. Simultaneously, the rise of do-it-yourself (DIY) 
solutions has empowered homeowners to take a more hands-on approach. 
These DIY methods vary, from transmitting data over Wi-Fi to integrating 
with home automation systems like Home Assistant, enabling a more 
efficient and responsive home environment.

This project uses an ESP32 microcontroller
and the ESP-IDF extension which is built on a real-time operating system (RTOS) to measure 
electricity consumption through the Han/P1 port. The collected data will be displayed on the built-in Oled display and 
transmitted using LoRaWAN to TTN and then to datacake.

## About the project

Heltec ESP32 LoRa is a microcontroller board based on the ESP32, with built-in LoRa and 0.96 inch OLED Display. It is a perfect solution for IoT low power consumption and long range.

To understand the data from the meter, check this: [Meter data: content of the data](https://www.kode24.no/guider/smart-meter-part-1-getting-the-meter-data/71287300). This explains the data, which is needed to understand what content of the data to read. In this project we read how much Watt's are used in real-time.

## Components of the project

### Hardware components
- Heltec Lora32 v2: The ESP32 microcontroller
- breadboard
- RJ12 cable(To the swedish electricity meter)
- Jumper wires
- The electricity meter works as power supply. (This project is built for the Aidon 6000-serie)

### Software components
- Espressif ESP32 Development Framework: install ESP-IDF through here, [ESP-IDF for ESP32](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/index.html)
- ESP-IDF Extension in Visual Studio Code, ESP-IDF has many example projects.
- ESP-IDF One of the example projects is the `UART_RX_TXtask`. Which uses freertos to create a task that reads from the UART and writes to the UART. This project uses the RX part of that.
- ESP-IDF: Also has a project for the Oled display, this is the `i2c_oled` project which is also used and implemented in this project.

- LoraWan: Device library for ESP-IDF with SX127X Lora chips: [ttn-esp32](https://github.com/manuelbl/ttn-esp32) This library enables the use of LoraWAN which is needed for data-transmission.
- Drivers: Might be needed to connect device to computer in Visual Studio code(test without drivers first), [Drivers](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads)

## How to run the project, nessacy installations
