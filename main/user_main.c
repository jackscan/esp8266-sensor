/* I2C example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_sleep.h"

#include "driver/i2c.h"

#include "MQTTClient.h"

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;

static const char *TAG = "main";

// #define WIFI_SSID CONFIG_WIFI_SSID
// #define WIFI_PASS CONFIG_WIFI_PASSWORD

#define SCL_IO           4                  /*!< gpio number for I2C master clock */
#define SDA_IO           5                  /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM   I2C_NUM_0          /*!< I2C port number for master dev */

#define MCP9808_ADDR                        0x18
#define MCP9808_START
#define WRITE_BIT                           I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                            I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                        0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                             0x0              /*!< I2C ack value */
#define NACK_VAL                            0x1              /*!< I2C nack value */
#define LAST_NACK_VAL                       0x2              /*!< I2C last_nack value */

#define MCP9808_REG_CONFIG 0x01      ///< MCP9808 config register

#define MCP9808_REG_CONFIG_SHUTDOWN 0x0100   ///< shutdown config
#define MCP9808_REG_CONFIG_CRITLOCKED 0x0080 ///< critical trip lock
#define MCP9808_REG_CONFIG_WINLOCKED 0x0040  ///< alarm window lock
#define MCP9808_REG_CONFIG_INTCLR 0x0020     ///< interrupt clear
#define MCP9808_REG_CONFIG_ALERTSTAT 0x0010  ///< alert output status
#define MCP9808_REG_CONFIG_ALERTCTRL 0x0008  ///< alert output control
#define MCP9808_REG_CONFIG_ALERTSEL 0x0004   ///< alert output select
#define MCP9808_REG_CONFIG_ALERTPOL 0x0002   ///< alert output polarity
#define MCP9808_REG_CONFIG_ALERTMODE 0x0001  ///< alert output mode

#define MCP9808_REG_UPPER_TEMP 0x02   ///< upper alert boundary
#define MCP9808_REG_LOWER_TEMP 0x03   ///< lower alert boundery
#define MCP9808_REG_CRIT_TEMP 0x04    ///< critical temperature
#define MCP9808_REG_AMBIENT_TEMP 0x05 ///< ambient temperature
#define MCP9808_REG_MANUF_ID 0x06     ///< manufacture ID
#define MCP9808_REG_DEVICE_ID 0x07    ///< device ID
#define MCP9808_REG_RESOLUTION 0x08   ///< resolutin

/**
 * @brief i2c master initialization
 */
static esp_err_t master_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = SDA_IO;
    conf.sda_pullup_en = 0;
    conf.scl_io_num = SCL_IO;
    conf.scl_pullup_en = 0;
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return ESP_OK;
}

static esp_err_t mcp9808_write16(uint8_t reg_address, uint16_t data)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MCP9808_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t)(data >> 8), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t)(data & 0xFF), ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t mcp9808_read16(uint8_t reg_address, uint16_t *data)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MCP9808_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MCP9808_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, ((uint8_t*)data) + 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, (uint8_t*)data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t mcp9808_config_set(uint16_t mask, bool enable)
{
    uint16_t conf = 0;
    if (~mask) ESP_ERROR_CHECK(mcp9808_read16(MCP9808_REG_CONFIG, &conf));
    if (enable) conf |= mask;
    else conf &= ~mask;
    ESP_ERROR_CHECK(mcp9808_write16(MCP9808_REG_CONFIG, conf));
    return ESP_OK;
}

static esp_err_t mcp9808_init(void)
{
    // XXX: Do we need a delay?
    vTaskDelay(100 / portTICK_RATE_MS);
    master_init();

    uint16_t mid = 0;
    ESP_ERROR_CHECK(mcp9808_read16(MCP9808_REG_MANUF_ID, &mid));
    ESP_LOGI(TAG, "manufacture id: %#x", mid);

    uint16_t devid = 0;
    ESP_ERROR_CHECK(mcp9808_read16(MCP9808_REG_DEVICE_ID, &devid));
    ESP_LOGI(TAG, "device id: %#x", devid);

    return mcp9808_config_set(0xFFFF, false);
}

static esp_err_t mcp9808_read_temperature(int16_t *temp)
{
    uint16_t t = 0;
    esp_err_t res = mcp9808_read16(MCP9808_REG_AMBIENT_TEMP, &t);
    if (res != ESP_OK) return res;
    *temp = (int16_t)(t & 0xFFF) - (int16_t)(t & 0x1000);
    return ESP_OK;
}

static void mqtt_client_thread(void *pvParameters)
{
    MQTTClient client;
    Network network;
    int rc = 0;
    char clientID[32] = {0};
    bool sensor_ready = false;

    ESP_LOGI(TAG, "ssid:%s pub:%s pubinterval:%u",
             CONFIG_WIFI_SSID, CONFIG_MQTT_PUB_TOPIC,
             CONFIG_MQTT_PUBLISH_INTERVAL);

    ESP_LOGI(TAG, "ver:%u clientID:%s keepalive:%d session:%d level:%u",
             CONFIG_DEFAULT_MQTT_VERSION, CONFIG_MQTT_CLIENT_ID,
             CONFIG_MQTT_KEEP_ALIVE, CONFIG_DEFAULT_MQTT_SESSION, CONFIG_DEFAULT_MQTT_SECURITY);

    ESP_LOGI(TAG, "broker:%s port:%u", CONFIG_MQTT_BROKER, CONFIG_MQTT_PORT);

    ESP_LOGI(TAG, "sendbuf:%u recvbuf:%u sendcycle:%u recvcycle:%u",
             CONFIG_MQTT_SEND_BUFFER, CONFIG_MQTT_RECV_BUFFER,
             CONFIG_MQTT_SEND_CYCLE, CONFIG_MQTT_RECV_CYCLE);

    MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;

    NetworkInit(&network);

    if (MQTTClientInit(&client, &network, 0, NULL, 0, NULL, 0) == false) {
        ESP_LOGE(TAG, "mqtt init err");
        vTaskDelete(NULL);
    }

    for (;;) {
        ESP_LOGI(TAG, "wait wifi connect...");
        xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);

        if ((rc = NetworkConnect(&network, CONFIG_MQTT_BROKER, CONFIG_MQTT_PORT)) != 0) {
            ESP_LOGE(TAG, "Return code from network connect is %d", rc);
            continue;
        }

        connectData.MQTTVersion = CONFIG_DEFAULT_MQTT_VERSION;

        sprintf(clientID, "%s_%u", CONFIG_MQTT_CLIENT_ID, esp_random());

        connectData.clientID.cstring = clientID;
        connectData.keepAliveInterval = CONFIG_MQTT_KEEP_ALIVE;

        // connectData.username.cstring = CONFIG_MQTT_USERNAME;
        // connectData.password.cstring = CONFIG_MQTT_PASSWORD;

        connectData.cleansession = CONFIG_DEFAULT_MQTT_SESSION;

        ESP_LOGI(TAG, "MQTT Connecting");

        if ((rc = MQTTConnect(&client, &connectData)) != 0) {
            ESP_LOGE(TAG, "Return code from MQTT connect is %d", rc);
            network.disconnect(&network);
            continue;
        }

        ESP_LOGI(TAG, "MQTT Connected");

#if defined(MQTT_TASK)

        if ((rc = MQTTStartTask(&client)) != pdPASS) {
            ESP_LOGE(TAG, "Return code from start tasks is %d", rc);
        } else {
            ESP_LOGI(TAG, "Use MQTTStartTask");
        }

#endif

        for (;;) {
            if (! sensor_ready && mcp9808_init() == ESP_OK) {
                sensor_ready = true;
            }

            int16_t temp = 0;
            if (!sensor_ready || mcp9808_read_temperature(&temp) != ESP_OK) {
                sensor_ready = false;
                // wait 10s before retrying
                vTaskDelay(10000 / portTICK_PERIOD_MS);
                continue;
            }

            char buf[16];
            snprintf(buf, sizeof(buf), "%d.%02d", temp >> 4, (temp & 0xF) * 100 / 16);

            ESP_LOGI(TAG, "temperature: %s", buf);

            MQTTMessage message = {
                .qos = 0,
                .retained = 1,
                .payload = buf,
                .payloadlen = strlen(buf),
            };

            if ((rc = MQTTPublish(&client, CONFIG_MQTT_PUB_TOPIC, &message)) != 0) {
                ESP_LOGE(TAG, "Return code from MQTT publish is %d", rc);
            } else {
                ESP_LOGI(TAG, "MQTT published topic %s, len:%u heap:%u",
                         CONFIG_MQTT_PUB_TOPIC,
                         message.payloadlen, esp_get_free_heap_size());
            }

            if (rc != 0) {
                break;
            }

            vTaskDelay(CONFIG_MQTT_PUBLISH_INTERVAL * 1000 / portTICK_PERIOD_MS);
        }

        network.disconnect(&network);
    }

    ESP_LOGW(TAG, "mqtt_client_thread going to be deleted");
    vTaskDelete(NULL);
    return;
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

void app_main(void)
{
    ESP_LOGI(TAG, "reset: %#x", esp_reset_reason());

    // Initialize NVS
    {
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);
    }

    initialise_wifi();

    int ret = xTaskCreate(&mqtt_client_thread, "sensor_thread", 4096, NULL, 8, NULL);

    if (ret != pdPASS)  {
        ESP_LOGE(TAG, "mqtt create client thread failed");
    }
}