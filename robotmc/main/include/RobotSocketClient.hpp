#pragma once

#include <functional>
#include <string>
#include "esp_timer.h"

#include "messages.hpp"
#include "SocketClient.hpp"

using std::array;
using std::function;
using std::string;

class RobotSocketClient : public SocketClient
{

public:
  Data readData()
  {
    array<uint8_t, 12> data = {};
    const SocketError err = readNbytes(data.data(), 12);
    if (err != SocketError::OK)
    {
      return noDataC;
    }

    float vr = 0.0;
    float vl = 0.0;

    memcpy(&vr, data.data() + 4, 4);
    memcpy(&vl, data.data() + 8, 4);

    return Data(vr, vl);
  }

  bool sendTextMessage(TextMessage msg)
  {
    string tmp(msg.msg);
    const int length = tmp.length();
    const int total_size = 8 + length + 1;
    uint8_t data[total_size] = {};

    const int tmp_id = htonl(msg.id);
    const int tmp_length = htonl(length);
    memcpy(data, &tmp_id, 4);
    memcpy(data + 4, &tmp_length, 4);
    memcpy(data + 8, tmp.c_str(), length);
    data[8 + length] = msg.rssi;

    return writeNbytes(data, total_size) == SocketError::OK;
  }

  bool sendHeartbeat(Heartbeat hb)
  {
    const int total_size = 5;
    uint8_t data[total_size] = {};

    const int tmp_id = htonl(hb.id);
    memcpy(data, &tmp_id, 4);
    data[4] = (uint8_t)hb.rssi;

    return writeNbytes(data, total_size) == SocketError::OK;
  }

  bool sendWhoAmI(WhoAmI wai)
  {
    const int total_size = 8;
    uint8_t data[total_size] = {};

    const int tmp_id = htonl(wai.id);
    const int tmp_uid = htonl(wai.uid);
    memcpy(data, &tmp_id, 4);
    memcpy(data + 4, &tmp_uid, 4);

    return writeNbytes(data, total_size) == SocketError::OK;
  }

  bool sendHeartbeatIfMust(Heartbeat hb)
  {
    if (getIdleTime() >= 1000000)
    {
      return sendHeartbeat(hb);
    }
    return false;
  }
};
