#include <string>

#include <array>
#include <span>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "RobotControl2Wv2.hpp"
#include "MotorControlv2.hpp"
#include "WiFiStation.hpp"
#include "UDPSocket.hpp"
#include "LEDWS2812.hpp"
#include "utils.hpp"
#include "DIO.hpp"
#include "Messages.hpp"

using std::array;
using std::span;
using std::to_string;

static constexpr uint8_t ROBOT_ID = 1;
static constexpr int BUFFER_SIZE = 1 * 1024;

void connectAndControlTask(void *param)
{
    UDPSocket s;
    if (s.createUDPSocket() != SockErr::ERR_OK)
    {
        ESP_LOGE(pcTaskGetName(NULL), "Could not create UDP socket");
        vTaskDelete(NULL);
        return;
    };

    array<uint8_t, BUFFER_SIZE> buffer;
    RobotControl2Wv2 wheels(10000000, 100000, GPIO_NUM_5, GPIO_NUM_4, GPIO_NUM_15, GPIO_NUM_14);
    while (true)
    {
        ESP_LOGI(pcTaskGetName(NULL), "Loop");
        if (!WiFiStation::getGatewayIP())
        {
            delay(1000);
            wheels.set_zero();
            continue;
        }

        if (!s.isConnected())
        {
            wheels.set_zero();
            ESP_LOGI(pcTaskGetName(NULL), "Trying to connect to PC...");
            const bool result = s.connect(WiFiStation::getGatewayIP(), 8080) == SockErr::ERR_OK;
            if (!result)
            {
                ESP_LOGE(pcTaskGetName(NULL), "\tConnection failed");
                continue;
            }
            ESP_LOGI(pcTaskGetName(NULL), "Connected to PC!");
            s.write(WhoAmI(ROBOT_ID).toBytes().data(), WhoAmI::msg_size);
        }

        if (s.idleTx() >= 1.f)
        {
            s.write(Heartbeat(WiFiStation::rssi(), ROBOT_ID).toBytes().data(), Heartbeat::msg_size);
            ESP_LOGI(pcTaskGetName(NULL), "HB");
        }

        if (!s.readReady(500))
        {
            wheels.set_zero();
            continue;
        }

        RCV incomingData = s.read(buffer.data(), (int)buffer.size());
        if (incomingData.count < 4)
        {
            ESP_LOGE(pcTaskGetName(NULL), "Too few data: %d", incomingData.count);
            continue;
        }

        uint32_t msg_id = 0;
        memcpy(&msg_id, buffer.data(), 4);
        msg_id = htonl(msg_id);

        ESP_LOGI(pcTaskGetName(NULL), "Got msg_id: %lu", msg_id);
        if (msg_id == ControlData::id)
        {
            optional<ControlData> data = ControlData::fromBuffer(span<uint8_t>(buffer).subspan(0, incomingData.count));
            if (!data)
            {
                wheels.set_zero();
                continue;
            }
            ESP_LOGI(pcTaskGetName(NULL), "vr: %ld | vl: %ld", data.value().vr, data.value().vl);
            wheels.set_vr((int)data.value().vr);
            wheels.set_vl((int)data.value().vl);
        }
        else if (msg_id == LEDData::id)
        {
            optional<LEDData> data = LEDData::fromBuffer(span<uint8_t>(buffer).subspan(0, incomingData.count));
            if (!data)
                continue;
            ESP_LOGI(pcTaskGetName(NULL), "GPIO NUM: %ld | (%i, %i, %i)", data.value().gpio_num, data.value().r, data.value().g, data.value().b);
            LEDWS2812 led((gpio_num_t)data.value().gpio_num);
            led.set(data.value().r, data.value().g, data.value().b);
            delay(1);
            DIO::setPullMode((gpio_num_t)data.value().gpio_num, GPIO_PULLDOWN_ONLY);
        }
        else if (msg_id == RequestWhoAmI::id)
        {
            optional<RequestWhoAmI> data = RequestWhoAmI::fromBuffer(span<uint8_t>(buffer).subspan(0, incomingData.count));
            if (!data)
                continue;
            s.write(WhoAmI(ROBOT_ID).toBytes().data(), WhoAmI::msg_size);
        }
        else
        {
            ESP_LOGE(pcTaskGetName(NULL), "Unknown msg id: %lu | Closing connection", msg_id);
            s.closeSocket();
            wheels.set_zero();
        }
    }
}

void test_task(void *param)
{
    RobotControl2Wv2 wheels(10000000, 100000, GPIO_NUM_5, GPIO_NUM_4, GPIO_NUM_15, GPIO_NUM_14);
    while (true)
    {
        ESP_LOGI(pcTaskGetName(NULL), "set_vr forw");
        wheels.set_zero();
        wheels.set_vr(60);
        delay(2000);

        ESP_LOGI(pcTaskGetName(NULL), "set_vr backw");
        wheels.set_zero();
        wheels.set_vr(-60);
        delay(2000);

        ESP_LOGI(pcTaskGetName(NULL), "set_vl forw");
        wheels.set_zero();
        wheels.set_vl(60);
        delay(2000);

        ESP_LOGI(pcTaskGetName(NULL), "set_vl backw");
        wheels.set_zero();
        wheels.set_vl(-60);
        delay(2000);
    }
}

extern "C" void app_main(void)
{
    WiFiStation::init(("Robot " + to_string(ROBOT_ID)).c_str(), "Hmm2", "ti pota exei? tipota");
    WiFiStation::startDefaultWiFiConnectionTask();
    xTaskCreate(connectAndControlTask, "C2Task", 1024 * 5, 0, ESP_TASK_TCPIP_PRIO, 0);
    // xTaskCreate(test_task, "test_task", 1024 * 5, 0, ESP_TASK_TCPIP_PRIO, 0);
}
