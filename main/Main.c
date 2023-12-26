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

#include "esp_efuse.h"
#include <esp_efuse_table.h>
#include "soc/rtc.h" //real time clock
#include "esp_pm.h"  //power management
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_heap_caps.h"

#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "Define/DeviceCode.h"
#include "Define/FunctionDefine.h"
#include <lwip/sockets.h>
#include "esp_websocket_client.h"
#include "esp_event.h"
#include "../../../../../esp/esp-idf/components/freertos/include/freertos/timers.h"

static const int RX_BUF_SIZE = 2048;

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)

static const char *TASK_TAG = "WEBSOCKET";
#define NO_DATA_TIMEOUT_SEC 10
const char *ssid = "KOVIS_302_2G";
const char *pass = "0233973563!";

static TimerHandle_t shutdown_signal_timer;
static SemaphoreHandle_t shutdown_sema;


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
    // uint64_t chip_id = ESP.get_efuse_mac();
    // printf("ESP32 Chip ID: %llx\n", chip_id);
}

/// @brief UART COMMUNICATION
/// @param logName
/// @param data
/// @return
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
    TASK_TAG = "TX_TASK";
    esp_log_level_set(TASK_TAG, ESP_LOG_INFO);
    while (1)
    {
        sendData(TASK_TAG, "Hello world");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}
static void rx_task(void *arg)
{
    TASK_TAG = "RX_TASK";
    esp_log_level_set(TASK_TAG, ESP_LOG_INFO);
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    while (1)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 500 / portTICK_PERIOD_MS);
        if (rxBytes > 0)
        {
            data[rxBytes] = 0;
            ESP_LOGI(TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
        }
    }
    free(data);
}

/// @brief DEVICE MAIN FUNCTION
/// @param
uint64_t READ_CHIP_ID()
{
    // Set up SPI bus and initialize the external SPI Flash chip
    uint64_t chip_id;
    esp_err_t err = esp_efuse_mac_get_default(&chip_id);
    return chip_id;
}
void READ_MODULE_INFOR()
{
    TASK_TAG = "READ_MODULE_INFOR";
    esp_log_level_set(TASK_TAG, ESP_LOG_INFO);
    uint8_t content = 0x00;
    uint8_t CheckSum[] = {0xAA, 0x55, (uint8_t)(sizeof(_READ_INFOR_MODULE) / sizeof(_READ_INFOR_MODULE) + sizeof(content) / sizeof(uint8_t)), _READ_INFOR_MODULE, content};
    uint8_t byteArray[] = {0xAA, 0x55, sizeof(_READ_INFOR_MODULE) / sizeof(_READ_INFOR_MODULE) + sizeof(content) / sizeof(uint8_t), _READ_INFOR_MODULE, content, CHECKSUM_CRC(CheckSum, sizeof(CheckSum)), 0x5A};
    //{AA}{55}{02}{4E}{00}{B1}{5A}
    sendByte(TASK_TAG, byteArray, sizeof(byteArray));
}
void READ_MODULE_RUNNING_INFOR()

{
    TASK_TAG = "READ_MODULE_RUNNING_INFOR";
    esp_log_level_set(TASK_TAG, ESP_LOG_INFO);
    uint8_t content = 0x00;
    uint8_t CheckSum[] = {0xAA, 0x55, (uint8_t)(sizeof(_RUNNING_INFOR_MODULE) / sizeof(_RUNNING_INFOR_MODULE) + sizeof(content) / sizeof(uint8_t)), _RUNNING_INFOR_MODULE, content};
    uint8_t byteArray[] = {0xAA, 0x55, sizeof(_RUNNING_INFOR_MODULE) + sizeof(content), _RUNNING_INFOR_MODULE, content, CHECKSUM_CRC(CheckSum, sizeof(CheckSum)), 0x5A};
    sendByte(TASK_TAG, byteArray, sizeof(byteArray));
}
void SET_MODULE_OPEATION(uint8_t type)
{
    TASK_TAG = "SET_MODULE_OPEATION";
    esp_log_level_set(TASK_TAG, ESP_LOG_INFO);

    uint8_t CheckSum[] = {0xAA, 0x55, (uint8_t)(sizeof(_SET_MODULE_OPERATION) / sizeof(_SET_MODULE_OPERATION) + sizeof(type) / sizeof(uint8_t)), _SET_MODULE_OPERATION, type};
    uint8_t byteArray[] = {0xAA, 0x55, sizeof(_SET_MODULE_OPERATION) + sizeof(type), _SET_MODULE_OPERATION, type, CHECKSUM_CRC(CheckSum, sizeof(CheckSum)), 0x5A};
    sendByte(TASK_TAG, byteArray, sizeof(byteArray));
}

//// @brief SOCKET SERVER

static void shutdown_signaler(TimerHandle_t xTimer)
{
    ESP_LOGI(TASK_TAG, "No data received for %d seconds, signaling shutdown", NO_DATA_TIMEOUT_SEC);
    xSemaphoreGive(shutdown_sema);
}
#if CONFIG_WEBSOCKET_URI_FROM_STDIN
static void get_string(char *line, size_t size)
{
    int count = 0;
    while (count < size)
    {
        int c = fgetc(stdin);
        if (c == '\n')
        {
            line[count] = '\0';
            break;
        }
        else if (c > 0 && c < 127)
        {
            line[count] = c;
            ++count;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
#endif /* CONFIG_WEBSOCKET_URI_FROM_STDIN */
static void websocket_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
    switch (event_id)
    {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TASK_TAG, "WEBSOCKET_EVENT_CONNECTED");
        break;
    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGI(TASK_TAG, "WEBSOCKET_EVENT_DISCONNECTED");
        break;
    case WEBSOCKET_EVENT_DATA:
        ESP_LOGI(TASK_TAG, "WEBSOCKET_EVENT_DATA");
        ESP_LOGI(TASK_TAG, "Received opcode=%d", data->op_code);
        ESP_LOGW(TASK_TAG, "WEBSOCKET Received=%.*s", data->data_len, (char *)data->data_ptr);
        ESP_LOGW(TASK_TAG, "Total payload length=%d, data_len=%d, current payload offset=%d\r\n", data->payload_len, data->data_len, data->payload_offset);

        xTimerReset(shutdown_signal_timer, portMAX_DELAY);
        break;
    case WEBSOCKET_EVENT_ERROR:
        ESP_LOGI(TASK_TAG, "WEBSOCKET_EVENT_ERROR");
        break;
    }
}
static void websocket_app_start(void)
{
    esp_websocket_client_config_t websocket_cfg = {};

    shutdown_signal_timer = xTimerCreate("Websocket shutdown timer", NO_DATA_TIMEOUT_SEC * 1000 / portTICK_PERIOD_MS,
                                         pdFALSE, NULL, shutdown_signaler);
    shutdown_sema = xSemaphoreCreateBinary();

#if CONFIG_WEBSOCKET_URI_FROM_STDIN
    char line[128];

    ESP_LOGI(TASK_TAG, "Please enter uri of websocket endpoint");
    get_string(line, sizeof(line));

    websocket_cfg.uri = line;
    ESP_LOGI(TASK_TAG, "Endpoint uri: %s\n", line);

#else
    websocket_cfg.uri = "ws://echo.websocket.events";

#endif /* CONFIG_WEBSOCKET_URI_FROM_STDIN */

    ESP_LOGI(TASK_TAG, "Connecting to %s...", websocket_cfg.uri);

    esp_websocket_client_handle_t client = esp_websocket_client_init(&websocket_cfg);
    esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY, websocket_event_handler, (void *)client);

    esp_websocket_client_start(client);
    xTimerStart(shutdown_signal_timer, portMAX_DELAY);
    char data[32];
    int i = 0;
    while (i < 10)
    {
        if (esp_websocket_client_is_connected(client))
        {
            int len = sprintf(data, "ESP32 Send %04d", i++);
            ESP_LOGI(TASK_TAG, "Sending %s", data);
            esp_websocket_client_send(client, data, len, portMAX_DELAY);
        }
        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    xSemaphoreTake(shutdown_sema, portMAX_DELAY);
    esp_websocket_client_stop(client);
    ESP_LOGI(TASK_TAG, "Websocket Stopped");
    esp_websocket_client_destroy(client);
}
int retry_num = 0;
static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_STA_START)
    {
        printf("WIFI CONNECTING....\n");
    }
    else if (event_id == WIFI_EVENT_STA_CONNECTED)
    {
        printf("WiFi CONNECTED\n");
    }
    else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        printf("WiFi lost connection\n");
        if (retry_num < 5)
        {
            esp_wifi_connect();
            retry_num++;
            printf("Retrying to Connect...\n");
        }
    }
    else if (event_id == IP_EVENT_STA_GOT_IP)
    {
        printf("Wifi got IP...\n\n");
    }
}
void wifi_connection()
{
    //                          s1.4
    // 2 - Wi-Fi Configuration Phase
    esp_netif_init();
    esp_event_loop_create_default();     // event loop                    s1.2
    esp_netif_create_default_wifi_sta(); // WiFi station                      s1.3
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation); //
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t wifi_configuration = {
        .sta = {
            .ssid = "",
            .password = "",

        }

    };
    strcpy((char *)wifi_configuration.sta.ssid, ssid);
    strcpy((char *)wifi_configuration.sta.password, pass);
    // esp_log_write(ESP_LOG_INFO, "Kconfig", "SSID=%s, PASS=%s", ssid, pass);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
    // 3 - Wi-Fi Start Phase
    esp_wifi_start();
    esp_wifi_set_mode(WIFI_MODE_STA);
    // 4- Wi-Fi Connect Phase
    esp_wifi_connect();
    printf("wifi_init_softap finished. SSID:%s  password:%s", ssid, pass);
}

/// @brief VOID MAIN PROGRAM
/// @param
void app_main(void)
{
    init();
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);

    uint64_t _chip_ID = READ_CHIP_ID();
    printf("ESP32 Chip ID: %llx\n", _chip_ID);

    ESP_LOGI(TASK_TAG, "[APP] Startup..");
    ESP_LOGI(TASK_TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TASK_TAG, "[APP] IDF version: %s", esp_get_idf_version());
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("WEBSOCKET_CLIENT", ESP_LOG_DEBUG);
    esp_log_level_set("TRANS_TCP", ESP_LOG_DEBUG);

    // ESP_ERROR_CHECK(nvs_flash_init());
    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());

    // /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
    //  * Read "Establishing Wi-Fi or Ethernet Connection" section in
    //  * examples/protocols/README.md for more information about this function.
    //  */
    // ESP_ERROR_CHECK(example_connect());
    nvs_flash_init();
    wifi_connection();
    websocket_app_start();
}
