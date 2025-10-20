#include <cstdint>

struct Heartbeat
{
    static const int id = 1000;
    int8_t rssi;

    Heartbeat(int8_t rssi) : rssi(rssi)
    {
    }
};

struct TextMessage
{
    static const int id = 1001;
    const char *msg;
    int8_t rssi;

    TextMessage(const char *msg, int8_t rssi) : msg(msg), rssi(rssi)
    {
    }
};

struct Data
{
    static const int id = 1002;
    float vr;
    float vl;
    bool valid;

    Data() : vr(0.f), vl(0.f), valid(false)
    {
    }

    Data(const float vr, const float vl) : vr(vr), vl(vl), valid(true)
    {
    }
};

struct WhoAmI
{
    static const int id = 1003;
    int uid = 1;
    WhoAmI(const int uid) : uid(uid)
    {
    }
};

const Data noDataC = Data();
