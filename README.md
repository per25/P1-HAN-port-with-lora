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

Despite these advancements, a critical limitation remains: the dependence on 
Wi-Fi for wireless data transmission. This reliance presents challenges, 
particularly in areas with poor or no Wi-Fi connectivity, limiting the scope of 
installation and use of such metering systems. Recognizing these issues, there 
is a clear need for an alternative solution that offers reliability and broader 
applicability in various environments.

To address these challenges we devoloped this project with Lora(Long Range) technology.
This project uses an ESP32 microcontroller
and the ESP-IDF extension which is built on a real-time operating system (RTOS) to measure 
electricity consumption through the Han/P1 port. The collected data will be displayed on the built-in 0.96 inch OLED Display and 
transmitted using LoRaWAN to TTN and then to datacake.

## About the project
This project is implemented in the C/C++ programming languages, offering a balance of performance and efficiency. The real-time capabilities provided by FreeRTOS ensure accurate and timely measurements, crucial for monitoring electricity consumption. The integration of LoRaWAN technology enables wireless communication, allowing users to remotely access and manage data through platforms like TTN and Datacake. The project showcases a comprehensive solution for smart metering, combining hardware components, software development with ESP-IDF, and wireless communication through LoRaWAN. This project empowers users to actively monitor and optimize their electricity usage in a user-friendly and efficient manner.

The project is built for the AIDON 6000-serie electricity meter, more specifically the AIDON 6534 which is a 3-phase energy service device for households. Link to AIDON meters: [Smart Energy Service Devices by Aidon](https://aidon.com/solutions/smart-energy-service-devices/)
(Note!, it should work for other meters with the same specifications. It is not tested woth other meters so we can not guarantee it!)

The idea with the project is that there should be an open-source project to read data from the meter and send the data with lorawan that is reasonably easy for others to download and use, or to be inspired of for their own project. This project was a bit challening to build because it did not exist any open-source project for data-readings with LoraWAN. For a person with intrests in IoT(Internet of Things), embedded systems and software in general this project will be easy to understand and use. Hopefully it will be fairly easy for a person with less experince in this field too.

To understand the data from the meter, check this: [Meter data: content of the data](https://www.kode24.no/guider/smart-meter-part-1-getting-the-meter-data/71287300). This explains the data, which is needed to understand what content of the data to read. In this project we read how much Watt's are used in real-time and the utility meter gives us this measurement every 10 seconds.

This project connets the device to TTN(The Things Network) and then has a webhook to Datacake to display the data, every software enthusiast has problay worked with TTN and Datacake before. We use them in this project for the simplicity, it is not too hard to connect your device.
- Follow this guide on how to connect the heltec to TTN: [Connect to TTN](https://docs.heltec.org/en/node/esp32/esp32_general_docs/lorawan/connect_to_gateway.html)
- Follow this guide on how to create a device on Datacake and a webhook to TTN: [Get started with TTN and Datacake](https://www.youtube.com/watch?v=WGVFgYp3k3s)

## Components of the project

### Hardware Components
- Heltec Lora32 v2: The ESP32 microcontroller
- breadboard
- 6 pin RJ12 cable(To the swedish electricity meter)
- Jumper wires
- Resistor 10 000 Ohm
- The electricity meter works as power supply. (This project is built for the Aidon 6000-serie)

#### Hardware Connection
![Wire connection](wireingdiagram.png)


### Software Components
- Espressif ESP32 Development Framework: install ESP-IDF through here, [ESP-IDF for ESP32](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/index.html)
- This project uses the ESP-IDF Extension in Visual Studio Code, ESP-IDF has many example projects.
- ESP-IDF One of the example projects is the `UART_RX_TXtask`. Which uses freertos to create a task that reads from the UART and writes to the UART. This project uses the RX part of that.
- ESP-IDF: Also has a project for the Oled display, this is the `i2c_oled` project which is also used and implemented in this project.

- LoraWan: Library for LoraWan in ESP-IDF with SX127X Lora chips: [ttn-esp32](https://github.com/manuelbl/ttn-esp32) This library enables the use of LoraWAN which is needed for data-transmission to TTN.
- Drivers: Might be needed to connect device to computer in Visual Studio code(test without drivers first), [Drivers](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads)

## Necessary Installations & How to run the project
- Start with the ESP-IDF extension in VScode(Visual Studio Code), documentation under `Software Components`.
- Then clone this project to your computer and open it in VScode.
- This project is dependant of the `ttn-esp32` component, install the component from the `ttn-esp32` link under `Software Components`
  - Important to Note!, In VScode you will need to set the frequency for LoraWAN, in Europe it is about 868 MHz(the frequency could easily be found online). Open the command pallete and search for `SDK Configuration Editor(Menuconfig)`, select the one with ESP-IDF, then search for Lora and set the Freqeuncy according to your region(Europe, Asia, North America, etc..)
- Then follow these guides to connect your device to TTN and create a webhook to Datacake, start with TTN: [Connect to TTN](https://docs.heltec.org/en/node/esp32/esp32_general_docs/lorawan/connect_to_gateway.html) and then [Get started with TTN and Datacake](https://www.youtube.com/watch?v=WGVFgYp3k3s)
  - The recomendations is to start with TTN and check that the device is connected before starting with the webhook to Datacake!
  - Also remember fic the Payload formatters on TTN and Datcake, they are show below under `Payloads on TTN and Datacake`

- Then click `Build, Flash and Monitor device` or do them step by step, personal choice!



### Payload decoders needed on TTN and Datacake
#### Payload decoder for TTN
```Javascript
function decodeUplink(input) {
  // Check if input.bytes has at least 4 bytes
  if (input.bytes.length < 4) {
    return {
      data: {},
      warnings: [],
      errors: ["Input does not contain enough bytes"]
    };
  }

  // Convert the first 4 bytes of the array to a 32-bit unsigned integer
  var uint32 = (input.bytes[0] << 24) | (input.bytes[1] << 16) | (input.bytes[2] << 8) | input.bytes[3];

  // Ensure the number is treated as unsigned
  var decimalNumber = uint32 >>> 0;

  return {
    data: {
      // Return the decimal number
      value: decimalNumber
    },
    warnings: [],
    errors: []
  };
}
```

#### Payload decoder for Datacake
``` Javascript
function Decoder(payload, port) {
    var value = {};
    
    try {
        
        var uint32 = (payload[0] << 24) | (payload[1] << 16) | (payload[2] << 8) | payload[3];
        var decimalNumber = uint32 >>> 0;
        
        console.log(decimalNumber);
        value.WATT = decimalNumber

    } catch (e) {
        // Handle any decoding errors
        console.error("Error decoding payload:", e);
    }

    return value;
}
```
















