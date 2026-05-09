#include <optional>
#include <atomic>

#include "freertos/FreeRTOS.h"
#include "Worker.hpp"
#include "esp_log.h"
#include "WiFiStation.hpp"
#include "UDPSocket.hpp"
#include "Messages.hpp"
#include "callbacks.hpp"

using std::atomic;
using std::optional;

static constexpr int BUFFER_SIZE = 1 * 1024;
static constexpr int READ_TIMEOUT_MS = 200;
static constexpr int MC_CLOCK_RESOLUTION_HZ = 10000000;
static constexpr int MC_PWM_FREQ_HZ = 100000;

static constexpr TickType_t CONNECT_RETRY_MS = 1000;
static constexpr TickType_t HEARTBEAT_MS = 1000;

struct NetworkTaskExtraData
{
    Worker<UDPPacket<200>> *udpPacketWorker;
    Worker<ControlData> *controlDataWorker;

    atomic<uint8_t> *uid;
    atomic<bool> *hasUID;
};
void networkTask(void *arg)
{
    NetworkTaskExtraData *data = (NetworkTaskExtraData *)arg;
    UDPSocket s;
    ControlData zero;
    TickType_t lastHeartbeat = xTaskGetTickCount();
    while (true)
    {
        const uint32_t gateway = WiFiStation::getGatewayIP();
        if (!gateway)
        {
            data->controlDataWorker->submit(zero);
            s.closeSocket();
            taskDelayMillis(CONNECT_RETRY_MS);
            continue;
        }

        if (!s.isConnected())
        {
            data->controlDataWorker->submit(zero);
            if (!s.IsSocketValid())
                s.createUDPSocket();

            s.connect(gateway, 8080);
            taskDelayMillis(CONNECT_RETRY_MS);
            continue;
        }

        if (xTaskGetTickCount() - lastHeartbeat > pdMS_TO_TICKS(HEARTBEAT_MS))
        {
            if (data->hasUID->load())
                s.write(Heartbeat(WiFiStation::rssi(), data->uid->load()).toBytes());
            else
                s.write(RequestWhoAmI().toBytes());
            lastHeartbeat = xTaskGetTickCount();
        }

        if (!s.readReady(READ_TIMEOUT_MS))
        {
            data->controlDataWorker->submit(zero);
            continue;
        }
        UDPPacket<200> packet = s.read<200>();
        data->udpPacketWorker->submit(packet);
        taskDelayMillis(10);
    }
}

atomic<bool> hasUID{false};
atomic<uint8_t> uid{0};
RobotControl2Wv2 wheels(MC_CLOCK_RESOLUTION_HZ, MC_PWM_FREQ_HZ, GPIO_NUM_5, GPIO_NUM_4, GPIO_NUM_15, GPIO_NUM_14);

ControlDataCallbackExtraData controlDataCallbackExtraData{.wheelsControl = &wheels};
Worker<ControlData> controlDataWorker(controlDataCallback, &controlDataCallbackExtraData);

WhoAmICallbackExtraData whoAmICallbackExtraData{.uid = &uid, .hasUID = &hasUID};
Worker<WhoAmI> whoAmIWorker(whoAmICallback, &whoAmICallbackExtraData);

Worker<LEDData> ledDataWorker(ledDataCallback);

UDPPacketCallbackExtraData udpPacketCallbackExtraData{.controlDataWorker = &controlDataWorker, .whoAmIWorker = &whoAmIWorker, .ledDataWorker = &ledDataWorker};
Worker<UDPPacket<200>> udpPacketWorker(udpPacketCallback, &udpPacketCallbackExtraData);

NetworkTaskExtraData networkTaskExtraData{.udpPacketWorker = &udpPacketWorker, .controlDataWorker = &controlDataWorker, .uid = &uid, .hasUID = &hasUID};

extern "C" void app_main(void)
{
    WiFiStation::init("esp32", "amd-LOQ-15AHP10", "ti pota exei? tipota");
    WiFiStation::startDefaultWiFiConnectionTask();

    xTaskCreate(networkTask, "nTask", 1024 * 5, &networkTaskExtraData, 10, 0);

    controlDataWorker.start("controlDataWorker");
    whoAmIWorker.start("whoAmIWorker");
    ledDataWorker.start("ledDataWorker");
    udpPacketWorker.start("udpPacketWorker");
}
