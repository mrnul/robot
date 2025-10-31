#pragma once

#include <functional>
#include <string>
#include "esp_timer.h"

#include "messages.hpp"
#include "SocketClient.hpp"

using std::vector;
using std::function;
using std::string;

class RobotSocketClient : public SocketClient
{

public:
  Data readData()
  {
    vector<uint8_t> data(Data().size());
    const SocketError err = readNbytes(data.data(), data.size());
    if (err != SocketError::OK)
    {
      return noDataC;
    }

    int32_t vr = 0;
    int32_t vl = 0;

    memcpy(&vr, data.data() + 4, 4);
    memcpy(&vl, data.data() + 8, 4);

    return Data(ntohl(vr), ntohl(vl));
  }

  bool sendTextMessage(TextMessage msg)
  {
    return writeNbytes(msg.to_bytes().data(), msg.size()) == SocketError::OK;
  }

  bool sendHeartbeat(Heartbeat hb)
  {
    return writeNbytes(hb.to_bytes().data(), hb.size()) == SocketError::OK;
  }

  bool sendWhoAmI(WhoAmI wai)
  {
    return writeNbytes(wai.to_bytes().data(), wai.size()) == SocketError::OK;
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
