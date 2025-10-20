#pragma once

#include <cstring>

#include "esp_wifi.h"
#include "esp_task.h"
#include "driver/temperature_sensor.h"

#include "MutexGuard.hpp"
#include "utils.hpp"

class WiFiStation
{
private:
    inline static temperature_sensor_handle_t temp_sensor = nullptr;
    inline static uint32_t gateway_ip = 0;
    inline static SemaphoreHandle_t mutex = nullptr;
    inline static bool connected = false;

    static void defaultWiFiConnectionTask(void *not_used)
    {
        while (true)
        {
            delay(3000);
            ESP_LOGI(pcTaskGetName(NULL), "Loop");
            if (WiFiStation::getConnected())
                continue;
            ESP_LOGI(pcTaskGetName(NULL), "esp_wifi_connect");
            const int tmp = esp_wifi_connect();
            if (tmp != ESP_OK)
            {
                ESP_LOGI(pcTaskGetName(NULL), "\tesp_wifi_connect %d", tmp);
            }
        }
    }

    static void defaultEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
    {
        if (event_base == WIFI_EVENT)
        {
            if (event_id == WIFI_EVENT_STA_DISCONNECTED)
            {
                WiFiStation::setGatewayIP(0);
                WiFiStation::setConnected(false);
                ESP_LOGE(pcTaskGetName(NULL), "Disconnected");
            }
            else if (event_id == WIFI_EVENT_STA_CONNECTED)
            {
                WiFiStation::setConnected(true);
                ESP_LOGE(pcTaskGetName(NULL), "Connected");
            }
        }
        else if (event_base == IP_EVENT)
        {
            if (event_id == IP_EVENT_STA_GOT_IP)
            {
                ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
                WiFiStation::setGatewayIP(event->ip_info.gw.addr);
                ESP_LOGI(pcTaskGetName(NULL), "Connected!");
            }
        }
    }

    static void initWiFiSta(const char *station_name, const char *ssid, const char *pass, esp_event_handler_t eventHandler)
    {
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        ESP_ERROR_CHECK(esp_netif_set_hostname(esp_netif_create_default_wifi_sta(), station_name));

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
        

        ESP_ERROR_CHECK(
            esp_event_handler_instance_register(
                WIFI_EVENT,
                ESP_EVENT_ANY_ID,
                eventHandler,
                NULL,
                NULL));
        ESP_ERROR_CHECK(
            esp_event_handler_instance_register(
                IP_EVENT,
                ESP_EVENT_ANY_ID,
                eventHandler,
                NULL,
                NULL));

        wifi_config_t wifi_config = {};

        strcpy((char *)wifi_config.sta.ssid, ssid);
        strcpy((char *)wifi_config.sta.password, pass);

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(34));
        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    }

    static void initTemperatureSensor()
    {
        temperature_sensor_config_t temp_sensor_config;
        temp_sensor_config.range_min = -10;
        temp_sensor_config.range_max = 80;
        temp_sensor_config.clk_src = TEMPERATURE_SENSOR_CLK_SRC_DEFAULT;

        ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &WiFiStation::temp_sensor));
        ESP_ERROR_CHECK(temperature_sensor_enable(WiFiStation::temp_sensor));
    }

public:
    static void init(const char *station_name, const char *ssid, const char *pass, esp_event_handler_t eventHandler = WiFiStation::defaultEventHandler)
    {
        mutex = xSemaphoreCreateMutex();
        WiFiStation::initWiFiSta(station_name, ssid, pass, eventHandler);
        WiFiStation::initTemperatureSensor();
    }

    static void startDefaultWiFiConnectionTask()
    {
        xTaskCreate(WiFiStation::defaultWiFiConnectionTask, "defWiFiConTask", 1024 * 5, NULL, ESP_TASK_MAIN_PRIO, NULL);
    }

    static int8_t rssi()
    {
        wifi_ap_record_t wifidata;
        if (esp_wifi_sta_get_ap_info(&wifidata) != ESP_OK)
            return -100;
        return wifidata.rssi;
    }

    static float getTemperatureC()
    {
        float value = -100.0;
        ESP_ERROR_CHECK(temperature_sensor_get_celsius(WiFiStation::temp_sensor, &value));
        return value;
    }

    static void setGatewayIP(const uint32_t ip)
    {
        MutexGuard lock(mutex, "WiFiStation::setGatewayIP");
        gateway_ip = ip;
    }

    static uint32_t getGatewayIP()
    {
        MutexGuard lock(mutex, "WiFiStation::getGatewayIP");
        return gateway_ip;
    }

    static void setConnected(const bool val)
    {
        MutexGuard lock(mutex, "WiFiStation::setConnected");
        connected = val;
    }

    static bool getConnected()
    {
        MutexGuard lock(mutex, "WiFiStation::setConnected");
        return connected;
    }
};
