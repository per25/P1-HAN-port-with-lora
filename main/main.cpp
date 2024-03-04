/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

#include "oled.h"
}

#include "freertos/FreeRTOS.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include <string.h>

#include "TheThingsNetwork.h"

static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_13)

// AppEUI (sometimes called JoinEUI)
const char *appEui = "0000000000000000";
// DevEUI
const char *devEui = "70B3D57ED0064BB3";
// AppKey
const char *appKey = "CE03DB00D327F40F148E8D07FFC4CA0D";

// Pins and other resources
#define TTN_SPI_HOST SPI2_HOST
#define TTN_SPI_DMA_CHAN SPI_DMA_DISABLED
#define TTN_PIN_SPI_SCLK 5
#define TTN_PIN_SPI_MOSI 27
#define TTN_PIN_SPI_MISO 19
#define TTN_PIN_NSS 18
#define TTN_PIN_RXTX TTN_NOT_CONNECTED
#define TTN_PIN_RST 14
#define TTN_PIN_DIO0 26
#define TTN_PIN_DIO1 35

#define INTERVAL_TO_SEND 30
#define DISPLAY_QUEUE_LENGTH 4
#define LORA_QUEUE_LENGTH INTERVAL_TO_SEND + 2
#define ITEM_SIZE sizeof(int[4])

#define PACKAGE_SIZE 581

QueueHandle_t loraQueue = NULL;
QueueHandle_t displayQueue = NULL;

static TheThingsNetwork ttn;

const unsigned TX_INTERVAL = 30;

static char *text = "OLED STARTED";

void sendMessages(void *pvParameter)
{
    uint8_t itemToSend[4];
    uint32_t average = 0;

    while (1)
    {
        // check the queue size queue
        if (uxQueueMessagesWaiting(loraQueue) >= INTERVAL_TO_SEND)
        {
            // Get the next INTERVAL_TO_SEND items from the queue

            for (int i = 0; i < INTERVAL_TO_SEND; i++)
            {
                if (xQueueReceive(loraQueue, &itemToSend, portMAX_DELAY) == pdPASS)
                {
                    // convert the 4 bytes to a int and add it to the average
                    average += (itemToSend[0] << 24) | (itemToSend[1] << 16) | (itemToSend[2] << 8) | itemToSend[3];
                }
            }

            float newAverage = average / INTERVAL_TO_SEND;

            // convert the new average to a 4 byte array
            average = (int)newAverage;

            printf("Average: %lu\n", average);
            itemToSend[0] = (average >> 24) & 0xFF;
            itemToSend[1] = (average >> 16) & 0xFF;
            itemToSend[2] = (average >> 8) & 0xFF;
            itemToSend[3] = average & 0xFF;

            printf("Sending message...\n");
            TTNResponseCode res = ttn.transmitMessage(itemToSend, sizeof(itemToSend));
            printf(res == kTTNSuccessfulTransmission ? "Message sent.\n" : "Transmission failed.\n");
        }
        vTaskDelay(TX_INTERVAL * pdMS_TO_TICKS(1000));
    }
}

void messageReceived(const uint8_t *message, size_t length, ttn_port_t port)
{
    printf("Message of %d bytes received on port %d:", length, port);
    for (int i = 0; i < length; i++)
        printf(" %02x", message[i]);
    printf("\n");
}

void init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Fix so this works with the inverter
    // uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV);
}

static void rx_task(void *arg)
{
    // static const int WATT_INDEX = 78;
    static const char *RX_TASK_TAG = "RX_TASK";
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(1000);
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);

    uint8_t dataToSend[4] = {0, 0, 0, 0};
    while (1)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);

        if (rxBytes == -1)
        {
            ESP_LOGE(RX_TASK_TAG, "Error reading from UART");
            continue;
        }

        // if (rxBytes != PACKAGE_SIZE)
        // {
        //     ESP_LOGE(RX_TASK_TAG, "Received package size is not correct: %d", rxBytes);
        //     continue;
        // }

        if (rxBytes > 0)
        {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
            ESP_LOGI(RX_TASK_TAG, "Watt: %d", data[53]);
            ESP_LOGI(RX_TASK_TAG, "Watt: %d", data[54]);
            ESP_LOGI(RX_TASK_TAG, "Watt: %d", data[55]);
            ESP_LOGI(RX_TASK_TAG, "Watt: %d", data[56]);
            ESP_LOGI(RX_TASK_TAG, "Watt: %d", data[57]);
            ESP_LOGI(RX_TASK_TAG, "Watt: %d", data[58]);
            dataToSend[0] = data[55];
            dataToSend[1] = data[56];
            dataToSend[2] = data[57];
            dataToSend[3] = data[58];

            printf("Sending array: [%d, %d, %d, %d]\n", dataToSend[0], dataToSend[1], dataToSend[2], dataToSend[3]);

            if (xQueueSend(loraQueue, &dataToSend, xMaxBlockTime) != pdPASS)
            {
                ESP_LOGE(RX_TASK_TAG, "Could not send to the queue");
            }

            if (xQueueSend(displayQueue, &dataToSend, xMaxBlockTime) != pdPASS)
            {
                ESP_LOGE(RX_TASK_TAG, "Could not send to the queue");
            }
        }
    }
    free(data);
}

void oled_task(void *pvParameter)
{
    while (1)
    {

        if (uxQueueMessagesWaiting(displayQueue) > 0)
        {
            // get the message from the queue

            uint8_t values[4];

            if (xQueueReceive(displayQueue, &values, portMAX_DELAY) == pdPASS)
            {
                int wattValue = (values[0] << 24) | (values[1] << 16) | (values[2] << 8) | values[3];

                printf("Watt value: %d\n", wattValue);

                // Buffer size is increased to accommodate the integer value and potential text.
                // Adjust the buffer size according to your expected maximum integer length.
                char buffer[64]; // Ensure this is large enough to hold the combined text and number

                // Format the new string with the wattValue included
                sprintf(buffer, "Watt + %d", wattValue);
                
                // Display the formatted string
                display_oled(buffer);
            }

            vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
    }
}


extern "C" void app_main(void);
void app_main(void)
{
    esp_err_t err;
    // Initialize the GPIO ISR handler service
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    ESP_ERROR_CHECK(err);

    // Initialize the NVS (non-volatile storage) for saving and restoring the keys
    err = nvs_flash_init();
    ESP_ERROR_CHECK(err);

    // Initialize SPI bus
    spi_bus_config_t spi_bus_config;
    memset(&spi_bus_config, 0, sizeof(spi_bus_config));
    spi_bus_config.miso_io_num = TTN_PIN_SPI_MISO;
    spi_bus_config.mosi_io_num = TTN_PIN_SPI_MOSI;
    spi_bus_config.sclk_io_num = TTN_PIN_SPI_SCLK;
    err = spi_bus_initialize(TTN_SPI_HOST, &spi_bus_config, TTN_SPI_DMA_CHAN);
    ESP_ERROR_CHECK(err);

    loraQueue = xQueueCreate(LORA_QUEUE_LENGTH, ITEM_SIZE);
    displayQueue = xQueueCreate(DISPLAY_QUEUE_LENGTH, ITEM_SIZE);

    if (loraQueue == NULL)
    {
        printf("Error creating the queue\n");
        return;
    }

    // Configure the SX127x pins
    ttn.configurePins(TTN_SPI_HOST, TTN_PIN_NSS, TTN_PIN_RXTX, TTN_PIN_RST, TTN_PIN_DIO0, TTN_PIN_DIO1);

    // The below line can be commented after the first run as the data is saved in NVS
    ttn.provision(devEui, appEui, appKey);

    // Register callback for received messages
    ttn.onMessage(messageReceived);

    //    ttn.setAdrEnabled(false);
    //    ttn.setDataRate(kTTNDataRate_US915_SF7);
    //    ttn.setMaxTxPower(14);
    init();
    oled_init();

    xTaskCreate(oled_task, "oled_task", 1024 * 2, NULL, 5, NULL);

    printf("Joining...\n");
    if (ttn.join())
    {
        printf("Joined.\n");
        xTaskCreate(sendMessages, "send_messages", 1024 * 4, (void *)0, 3, nullptr);
        text = "Connected to TTN...";
    }
    else
    {
        printf("Join failed. Goodbye\n");
    }
    printf("Starting the rx tasks\n");
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
}
