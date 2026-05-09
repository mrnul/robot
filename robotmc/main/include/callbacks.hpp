#include <atomic>

#include "driver/gpio.h"
#include "Messages.hpp"
#include "RobotControl2Wv2.hpp"
#include "UDPSocket.hpp"
#include "LEDWS2812.hpp"

using std::atomic;

struct UDPPacketCallbackExtraData
{
    Worker<ControlData> *controlDataWorker;
    Worker<WhoAmI> *whoAmIWorker;
    Worker<LEDData> *ledDataWorker;
};
template <int N>
inline bool udpPacketCallback(UDPPacket<N> &incomingData, void *arg)
{
    UDPPacketCallbackExtraData *extra = (UDPPacketCallbackExtraData *)arg;

    if (incomingData.count < 4)
        return true;

    uint32_t msg_id = 0;
    memcpy(&msg_id, incomingData.buffer.data(), 4);
    msg_id = ntohl(msg_id);

    const span<uint8_t> payload = span<uint8_t>(incomingData.buffer).first(incomingData.count);

    ESP_LOGI(pcTaskGetName(NULL), "Got msg_id: %lu", msg_id);
    if (msg_id == ControlData::id)
    {
        optional<ControlData> data = ControlData::fromBuffer(payload);
        if (!data)
        {
            return true;
        }
        extra->controlDataWorker->submit(data.value());
    }
    else if (msg_id == LEDData::id)
    {
        optional<LEDData> data = LEDData::fromBuffer(payload);
        if (!data)
            return true;
        extra->ledDataWorker->submit(data.value());
    }
    else if (msg_id == WhoAmI::id)
    {
        optional<WhoAmI> data = WhoAmI::fromBuffer(payload);
        if (!data)
            return true;
        extra->whoAmIWorker->submit(data.value());
    }
    else
    {
        ESP_LOGE(pcTaskGetName(NULL), "Unknown msg id: %lu | Closing connection", msg_id);
    }
    return true;
}

struct ControlDataCallbackExtraData
{
    RobotControl2Wv2 *wheelsControl;
};
inline bool controlDataCallback(ControlData &data, void *arg)
{
    ControlDataCallbackExtraData *extra = (ControlDataCallbackExtraData *)arg;
    ESP_LOGI(pcTaskGetName(NULL), "vr: %ld | vl: %ld", data.vr, data.vl);
    extra->wheelsControl->setVr(data.vr);
    extra->wheelsControl->setVl(data.vl);
    return true;
}

inline bool ledDataCallback(LEDData &data, void *arg)
{
    ESP_LOGI(pcTaskGetName(NULL), "GPIO NUM: %ld | (%i, %i, %i) | %i", data.gpio_num, data.r, data.g, data.b, data.colorOrder);
    {
        LEDWS2812 led((gpio_num_t)data.gpio_num);
        if (data.colorOrder == ColorOrder::RGB)
            led.set(data.r, data.g, data.b);
        else if (data.colorOrder == ColorOrder::GRB)
            led.set(data.g, data.r, data.b);
    }
    gpio_set_pull_mode((gpio_num_t)data.gpio_num, GPIO_PULLDOWN_ONLY);
    return true;
}

struct WhoAmICallbackExtraData
{
    atomic<uint8_t> *uid;
    atomic<bool> *hasUID;
};
inline bool whoAmICallback(WhoAmI &data, void *arg)
{
    WhoAmICallbackExtraData *extra = (WhoAmICallbackExtraData *)arg;
    extra->uid->store(data.uid);
    extra->hasUID->store(true);
    return true;
}
