/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "DeviceCode.h"

static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)

void init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    printf("UART Init");
}

int sendData(const char *logName, const char *data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "ESP32 sent %d string", txBytes);
    return txBytes;
}
int sendByte(const char *logName, const uint8_t *data, uint8_t len)
{
    // const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "ESP32 sent %d bytes", txBytes);
    return txBytes;
}

uint8_t CHECKSUM_CRC(uint8_t *pBuf, uint16_t len)
{
    uint16_t checksum = 0;
    int16_t i;
    for (i = 0; i < len; i++)
    {
        checksum += *(pBuf + i);
    }
    return (256 - (checksum % 256));
}
static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1)
    {
        sendData(TX_TASK_TAG, "Hello world");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    while (1)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 500 / portTICK_PERIOD_MS);
        if (rxBytes > 0)
        {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
    }
    free(data);
}
/// @brief DEVICE MAIN FUNCTION
/// @param
void READ_MODULE_INFOR()
{
    static const char *TX_TASK_TAG = "READ_MODULE_INFOR";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    uint8_t content[] = {0x00};
    uint8_t CheckSum[] = {0xAA, 0x55, sizeof(_READ_INFOR_MODULE) + sizeof(content), _READ_INFOR_MODULE, 0x00};
    uint8_t byteArray[] = {0xAA, 0x55, 0x02, _READ_INFOR_MODULE, 0x00, CHECKSUM_CRC(CheckSum, sizeof(CheckSum)), 0x5A};
    sendByte(TX_TASK_TAG, byteArray, sizeof(byteArray));
}
void READ_MODULE_INFOR2()
{
    static const char *TX_TASK_TAG = "READ_MODULE_INFOR";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    uint8_t content[] = {0x00};
    uint8_t CheckSum[] = {0xAA, 0x55, sizeof(_READ_INFOR_MODULE) + sizeof(content), _READ_INFOR_MODULE, 0x00};
    uint8_t byteArray[] = {0xAA, 0x55, 0x02, _READ_INFOR_MODULE, 0x00, CHECKSUM_CRC(CheckSum, sizeof(CheckSum)), 0x5A};
    sendByte(TX_TASK_TAG, byteArray, sizeof(byteArray));
}
/// @brief VOID MAIN PROGRAM
/// @param

void app_main(void)
{
    init();
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
    // xTaskCreate(tx_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
    while (1)
    {
    READ_MODULE_INFOR();
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}
