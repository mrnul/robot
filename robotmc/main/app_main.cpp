#include <string>

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "RobotControl2Wv2.hpp"
#include "MotorControlv2.hpp"
#include "WiFiStation.hpp"
#include "RobotSocketClient.hpp"
#include "LEDWS2812.hpp"
#include "utils.hpp"
#include "DIO.hpp"

using std::to_string;

const int robot_id = 1;

void clientConnectionAndControlTask(void *param)
{
    RobotSocketClient client;
    RobotControl2Wv2 wheels(10000000, 100000, GPIO_NUM_5, GPIO_NUM_4, GPIO_NUM_15, GPIO_NUM_14);
    while (true)
    {
        wheels.set_zero();
        delay(1000);
        while (WiFiStation::getGatewayIP())
        {
            if (!client)
            {
                wheels.set_zero();
                ESP_LOGI(pcTaskGetName(NULL), "Trying to connect to PC...");
                const bool result = client.connect(WiFiStation::getGatewayIP(), 8080, 500) == SocketError::OK;
                if (!result)
                {
                    ESP_LOGE(pcTaskGetName(NULL), "\tConnection failed");
                    break;
                }
                client.sendWhoAmI(WhoAmI(robot_id));
                client.sendTextMessage(TextMessage("Hello", WiFiStation::rssi()));
                ESP_LOGI(pcTaskGetName(NULL), "Connected to PC!");
            }

            Data data = client.readData();
            if (data.vl != noDataC.vl || data.vr != noDataC.vr)
            {
                ESP_LOGI(pcTaskGetName(NULL), "vr: %ld | vl: %ld", data.vr, data.vl);
                wheels.set_vr((int)data.vr);
                wheels.set_vl((int)data.vl);
            }
            else
            {
                wheels.set_zero();
            }
            if (client.sendHeartbeatIfMust(Heartbeat(WiFiStation::rssi())))
            {
                ESP_LOGI(pcTaskGetName(NULL), "HB");
            }
        }
    }
}

extern "C" void app_main(void)
{
    {
        LEDWS2812 built_in_led;
        built_in_led.set(0, 0, 0);
        WiFiStation::init(("Robot " + to_string(robot_id)).c_str(), "Hmm2", "ti pota exei? tipota");
        WiFiStation::startDefaultWiFiConnectionTask();
        xTaskCreate(clientConnectionAndControlTask, "clientC2Task", 1024 * 5, 0, ESP_TASK_TCPIP_PRIO, 0);
        built_in_led.set(255, 0, 0);
        delay(200);
        built_in_led.set(0, 255, 0);
        delay(200);
        built_in_led.set(0, 0, 255);
    }
    {
        LEDWS2812 extra_led(GPIO_NUM_3);
        delay(200);
        // order here is G, R, B
        extra_led.set(0, 255, 0);
        delay(200);
        extra_led.set(255, 0, 0);
        delay(200);
        extra_led.set(0, 0, 255);
        delay(200);
        extra_led.set(0, 50, 0);
    }
    DIO::setPullMode(GPIO_NUM_3, GPIO_PULLDOWN_ONLY);
    DIO::setPullMode(GPIO_NUM_8, GPIO_PULLDOWN_ONLY);
}
