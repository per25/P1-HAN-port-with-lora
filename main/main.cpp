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
}

#include "freertos/FreeRTOS.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include <string.h>

#include "TheThingsNetwork.h"

static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_16)

// AppEUI (sometimes called JoinEUI)
const char *appEui = "0000000000000000";
// DevEUI
const char *devEui = "70B3D57ED0064BB4";
// AppKey
const char *appKey = "DE5BDD7FD069E275915D859BAA293B29";

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

#define QUEUE_LENGTH 5
#define ITEM_SIZE sizeof(int[4])

QueueHandle_t xQueue = NULL;

static TheThingsNetwork ttn;

const unsigned TX_INTERVAL = 30;
// static uint8_t msgData[] = "Hello, world";

void sendMessages(void *pvParameter)
{
    uint8_t itemToSend[4];
    while (1)
    {
        if (xQueueReceive(xQueue, &itemToSend, portMAX_DELAY) == pdPASS)
        {
            // Successfully received an item from the queue
            printf("Received array: [%d, %d, %d, %d]\n", itemToSend[0], itemToSend[1], itemToSend[2], itemToSend[3]);
        }
        printf("Sending message...\n");
        TTNResponseCode res = ttn.transmitMessage(itemToSend, sizeof(itemToSend));
        printf(res == kTTNSuccessfulTransmission ? "Message sent.\n" : "Transmission failed.\n");

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
}

static void rx_task(void *arg)
{
    static const int WATT_INDEX = 78;
    static const char *RX_TASK_TAG = "RX_TASK";
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(1000);
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);

    uint8_t dataToSend[4] = {0, 0, 0, 0};
    while (1)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0)
        {
            data[rxBytes] = 0;
            // ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
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

            if (xQueueSend(xQueue, &dataToSend, xMaxBlockTime) != pdPASS)
            {
                ESP_LOGE(RX_TASK_TAG, "Could not send to the queue");
            }
        }
    }
    free(data);
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

    xQueue = xQueueCreate(QUEUE_LENGTH, ITEM_SIZE);

    if (xQueue == NULL)
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

    printf("Joining...\n");
    if (ttn.join())
    {
        printf("Joined.\n");
        xTaskCreate(sendMessages, "send_messages", 1024 * 4, (void *)0, 3, nullptr);
    }
    else
    {
        printf("Join failed. Goodbye\n");
    }
    printf("Starting the tasks\n");
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
}