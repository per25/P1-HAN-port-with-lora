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

static const int RX_BUF_SIZE = 1024 * 2;

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

#define INTERVAL_TO_SEND 6 // 30
#define DISPLAY_QUEUE_LENGTH 4
#define LORA_QUEUE_LENGTH INTERVAL_TO_SEND + 2
#define ITEM_SIZE sizeof(int[4])

#define PACKAGE_SIZE 581

QueueHandle_t loraQueue = NULL;
QueueHandle_t displayQueue = NULL;

static TheThingsNetwork ttn;

const unsigned TX_INTERVAL = 30;

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

    // set internal pull up on RX pin
    ESP_ERROR_CHECK(gpio_set_pull_mode(RXD_PIN, GPIO_PULLUP_ONLY));

    // Fix so this works with the inverter
    ESP_ERROR_CHECK(uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV));

}

#include <ctime>

#define TIME_START_INDEX 31
#define STR_LEN 0x0C

struct PackageData
{
    uint16_t year;
    uint8_t month;
    uint8_t dayOfMonth;
    uint8_t dayOfWeek;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t hundredth;
    uint8_t deviation;
    uint8_t timeStatus;
};

/**
 * @brief Validates the time data and adjusts the average calculation if necessary.
 *
 * This function checks if the time stamp in the given data is in the specified time to see if any data is missing.
 * Should be called after the validatePackageFormat function, to avoid unnecessary calculations and array out of bounds errors.
 *
 * @param data Pointer to the data array.
 * @param packageData Reference to the PackageData struct to store the parsed time data.
 */
void validateTime(uint8_t *data, PackageData &packageData)
{
    if (data[TIME_START_INDEX] != STR_LEN)
    {
        printf("STR LEN is not correct, length is not correct\n");
        return;
    }

    packageData.year = (data[TIME_START_INDEX + 1] << 8) | data[TIME_START_INDEX + 2];
    packageData.month = data[TIME_START_INDEX + 3];
    packageData.dayOfMonth = data[TIME_START_INDEX + 4];
    packageData.dayOfWeek = data[TIME_START_INDEX + 5];
    packageData.hour = data[TIME_START_INDEX + 6];
    packageData.minute = data[TIME_START_INDEX + 7];
    packageData.second = data[TIME_START_INDEX + 8];
    packageData.hundredth = data[TIME_START_INDEX + 9]; // Not used ?
    packageData.deviation = data[TIME_START_INDEX + 10]; // Not used ?
    packageData.timeStatus = data[TIME_START_INDEX + 11]; // Should be the summer or winter time

    // Log the data to the console
    printf("Year: %d\n", packageData.year);
    printf("Month: %d\n", packageData.month);
    printf("Day of month: %d\n", packageData.dayOfMonth);
    printf("Day of week: %d\n", packageData.dayOfWeek);
    printf("Hour: %d\n", packageData.hour);
    printf("Minute: %d\n", packageData.minute);
    printf("Second: %d\n", packageData.second);
    printf("Hundredth: %d\n", packageData.hundredth);
    printf("Deviation: %d\n", packageData.deviation);
    printf("Time status: %d\n", packageData.timeStatus);
}

/**
 * @brief Checks the time difference between two packages.
 *
 * This function calculates the time difference in seconds between two packages based on their time stamps.
 *
 * @param previousPackageData Reference to the PackageData struct of the previous package.
 * @param currentPackageData Reference to the PackageData struct of the current package.
 * @return The time difference in seconds.
 */
int calculateTimeDifference(const PackageData &previousPackageData, const PackageData &currentPackageData)
{
    struct tm previousTime = {0};
    previousTime.tm_year = previousPackageData.year - 1900;
    previousTime.tm_mon = previousPackageData.month - 1;
    previousTime.tm_mday = previousPackageData.dayOfMonth;
    previousTime.tm_hour = previousPackageData.hour;
    previousTime.tm_min = previousPackageData.minute;
    previousTime.tm_sec = previousPackageData.second;

    struct tm currentTime = {0};
    currentTime.tm_year = currentPackageData.year - 1900;
    currentTime.tm_mon = currentPackageData.month - 1;
    currentTime.tm_mday = currentPackageData.dayOfMonth;
    currentTime.tm_hour = currentPackageData.hour;
    currentTime.tm_min = currentPackageData.minute;
    currentTime.tm_sec = currentPackageData.second;

    time_t previousTimestamp = mktime(&previousTime);
    time_t currentTimestamp = mktime(&currentTime);

    return difftime(currentTimestamp, previousTimestamp);
}

#define PACKAGE_LENGTH 581 // The length of the package should be 581 bytes if the format is correct
#define START_END_BYTE 0x7E // The start and end byte of the package
#define BYTE_53_VALUE 0xFF // The bytes befor the watt value should be 0xFF
#define BYTE_54_VALUE 0x06 // And 0X06 for the 53ht and 54th byte

/**
 * @brief Validates the format of a package.
 *
 * This function checks if the given data has the correct length and contains specific byte values at certain indices.
 *
 * @param data Pointer to the data array.
 * @param length Length of the data array.
 * @return True if the package format is valid, false otherwise.
 */
bool validatePackageFormat(uint8_t *data, size_t length)
{
    // Check if the length of the package is correct
    if (length != PACKAGE_LENGTH)
    {
        ESP_LOGE("RX_TASK_TAG", "Data is not correct, length is not correct");
        return false;
    }

    // Define the checks to be performed on the package
    struct {
        size_t index; // The index of the byte to check
        uint8_t expectedValue; // The expected value of the byte
    } checks[] = {
        {0, START_END_BYTE}, // The first byte should be START_END_BYTE
        {PACKAGE_LENGTH - 1, START_END_BYTE}, // The last byte should be START_END_BYTE
        {53, BYTE_53_VALUE},
        {54, BYTE_54_VALUE}
    };

    // Loop over the checks
    for (size_t i = 0; i < sizeof(checks) / sizeof(checks[0]); i++)
    {
        // If the byte at the specified index does not have the expected value
        if (data[checks[i].index] != checks[i].expectedValue)
        {
            ESP_LOGE("RX_TASK_TAG", "Data is not correct, byte at index %zu is not 0x%02X", checks[i].index, checks[i].expectedValue);
            return false;
        }
    }

    // If all checks passed, the package format is valid
    return true;
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(1000);
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);

    PackageData previousPackageData = {0};

    if (data == NULL)
    {
        ESP_LOGE(RX_TASK_TAG, "Failed to allocate memory");
        // Restart the device 
        esp_restart();
    }

    uint8_t dataToSend[4] = {0, 0, 0, 0};
    while (1)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);

        if (rxBytes == -1)
        {
            ESP_LOGE(RX_TASK_TAG, "Error reading from UART");
            continue;
        }

        if (rxBytes > 0)
        {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);

            // check if the data is valid
            if (!validatePackageFormat(data, rxBytes))
            {
                continue;
            }

            // get the time data from the package
            PackageData currentPackageData = {0};
            validateTime(data, currentPackageData);

            // If the time data is valid, calculate the time difference
            if (currentPackageData.year != 0)
            {
                int timeDifference = calculateTimeDifference(previousPackageData, currentPackageData);
                printf("Time difference: %d\n", timeDifference);

                // If the time difference is not 15 seconds, adjust the average calculation
                if (timeDifference > 15)
                {
                    // TODO Adjust the average calculation
                    printf("Time difference is not 15 seconds\n");
                }

                // Update the previous package data
                previousPackageData = currentPackageData;
            }

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
                uint32_t wattValue = (values[0] << 24) | (values[1] << 16) | (values[2] << 8) | values[3];

                printf("Watt value: %lu\n", wattValue);

                const int buffer_size = 64;
                // Buffer size is increased to accommodate the integer value and potential text.
                // Adjust the buffer size according to your expected maximum integer length.
                char buffer[buffer_size]; // Ensure this is large enough to hold the combined text and number

                // Format the new string with the wattValue included
                snprintf(buffer, buffer_size, "Watt : %lu", wattValue);

                // Display the formatted string
                display_oled(buffer);
            }
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}


extern "C" void app_main(void);
/**
 * @brief The main function of the application.
 *
 * This function is the entry point of the application. It initializes various components,
 * configures the SX127x pins, provisions the device with necessary keys, joins the network,
 * and starts the necessary tasks for sending and receiving messages.
 */
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

    if (loraQueue == NULL || displayQueue == NULL)
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

    init();
    oled_init();

    xTaskCreate(oled_task, "oled_task", 1024 * 2, NULL, 5, NULL);

    printf("Joining...\n");
    display_oled("System started \nConnecting to TTN");
    if (ttn.join())
    {
        printf("Joined.\n");
        xTaskCreate(sendMessages, "send_messages", 1024 * 4, (void *)0, 3, nullptr);
    }
    else
    {
        printf("Join failed. Goodbye\n");
    }
    printf("Starting the rx tasks\n");
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
}
