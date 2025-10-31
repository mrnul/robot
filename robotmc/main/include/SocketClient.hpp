#pragma once

#include <string>
#include <socket.h>

#include "esp_timer.h"
#include "MutexGuard.hpp"

using std::array;
using std::string;

enum class SocketError
{
  OK = 1,

  ERR_CREATE_SOCKET = -1,
  ERR_NONBLOCK_FAILED = -2,
  ERR_CONNECT_FAIL = -3,
  ERR_SELECT_ERROR = -4,
  ERR_CONNECT_TIMEOUT = -5,
  ERR_GETSOCKOPT = -6,
  ERR_CONNECT_REFUSED = -7,
  ERR_TCP_NODELAY = -8,
  ERR_NOT_CONNECTED_READ = -9,
  ERR_SELECT_READ = -10,
  ERR_READ_TIMEOUT = -11,
  ERR_READ_FAIL = -12,
  ERR_NOT_CONNECTED_WRITE = -13,
  ERR_SELECT_WRITE = -14,
  ERR_WRITE_TIMEOUT = -15,
  ERR_WRITE_FAIL = -16,
  ERR_INCOMPLETE_READ = -17,
  ERR_INCOMPLETE_WRITE = -18,
};

class SocketClient
{
private:
  SemaphoreHandle_t mutex;
  int64_t last_tx;
  int64_t last_rx;
  int sockfd;
  bool connected;

public:
  SocketClient()
      : mutex(xSemaphoreCreateMutex()), last_tx(0), last_rx(0), sockfd(-1), connected(false)
  {
  }

  SocketError connect(in_addr_t ip, const uint16_t port, int32_t timeout_ms)
  {
    MutexGuard lock(mutex, "SocketClient::connect");

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
      return SocketError::ERR_CREATE_SOCKET;
    }

    sockaddr_storage serveraddr = {};
    sockaddr_in *helper = (sockaddr_in *)&serveraddr;
    helper->sin_family = AF_INET;
    helper->sin_addr.s_addr = ip;
    helper->sin_port = htons(port);

    // make it non blocking
    int flags = lwip_fcntl(sockfd, F_GETFL, 0);
    if (lwip_fcntl(sockfd, F_SETFL, flags | O_NONBLOCK) < 0)
    {
      closeConnection();
      return SocketError::ERR_NONBLOCK_FAILED;
    }

    int res = ::connect(sockfd, (sockaddr *)&serveraddr, sizeof(serveraddr));
    if (res < 0 && errno != EINPROGRESS)
    {
      closeConnection();
      return SocketError::ERR_CONNECT_FAIL;
    }

    fd_set fdset;
    timeval tv;
    FD_ZERO(&fdset);
    FD_SET(sockfd, &fdset);
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    res = select(sockfd + 1, nullptr, &fdset, nullptr, timeout_ms < 0 ? nullptr : &tv);

    if (res < 0)
    { // some error
      closeConnection();
      return SocketError::ERR_SELECT_ERROR;
    }
    if (res == 0)
    { // timeout
      closeConnection();
      return SocketError::ERR_CONNECT_TIMEOUT;
    }

    int sockerr;
    socklen_t len = (socklen_t)sizeof(int);
    res = getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &sockerr, &len);
    if (res < 0)
    { // some error
      closeConnection();
      return SocketError::ERR_GETSOCKOPT;
    }

    if (sockerr != 0)
    { // some error
      closeConnection();
      return SocketError::ERR_CONNECT_REFUSED;
    }

    // set TCP_NODELAY
    int on = 1;
    res = setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(int));
    if (res < 0)
    {
      closeConnection();
      return SocketError::ERR_TCP_NODELAY;
    }
    connected = true;
    return SocketError::OK;
  }

  void closeConnection()
  {
    if (sockfd >= 0)
    {
      close(sockfd);
      sockfd = -1;
    }
    connected = false;
  }

  SocketError readNbytes(uint8_t *bytes, const int count, int32_t timeout_ms = 500)
  {
    MutexGuard lock(mutex, "SocketClient::readNbytes");

    if (!isConnected())
    {
      return SocketError::ERR_NOT_CONNECTED_READ;
    }

    int read_count = 0;

    fd_set read_fds;
    while (read_count < count)
    {
      FD_ZERO(&read_fds);
      FD_SET(sockfd, &read_fds);

      timeval tv;
      tv.tv_sec = timeout_ms / 1000;
      tv.tv_usec = (timeout_ms % 1000) * 1000;

      int sel = select(sockfd + 1, &read_fds, nullptr, nullptr, &tv);
      if (sel < 0)
      {
        closeConnection();
        return SocketError::ERR_SELECT_READ;
      }
      else if (sel == 0)
      {
        // Timeout
        return SocketError::ERR_READ_TIMEOUT;
      }

      // Socket is ready for reading
      int res = recv(sockfd, bytes + read_count, count - read_count, 0);
      if (res <= 0)
      {
        closeConnection();
        return SocketError::ERR_READ_FAIL;
      }

      read_count += res;
    }

    if (read_count != count)
    {
      return SocketError::ERR_INCOMPLETE_READ;
    }

    last_rx = esp_timer_get_time();
    return SocketError::OK;
  }

  SocketError writeNbytes(const uint8_t *bytes, const int count, int32_t timeout_ms = 1000)
  {
    MutexGuard lock(mutex, "SocketClient::writeNbytes");
    if (!isConnected())
    {
      return SocketError::ERR_NOT_CONNECTED_WRITE;
    }

    int write_count = 0;
    fd_set write_fds;

    while (write_count < count)
    {
      FD_ZERO(&write_fds);
      FD_SET(sockfd, &write_fds);

      timeval tv;
      tv.tv_sec = timeout_ms / 1000;
      tv.tv_usec = (timeout_ms % 1000) * 1000;

      int sel = select(sockfd + 1, nullptr, &write_fds, nullptr, &tv);
      if (sel < 0)
      {
        closeConnection();
        return SocketError::ERR_SELECT_WRITE;
      }
      else if (sel == 0)
      {
        // timeout
        return SocketError::ERR_WRITE_TIMEOUT;
      }

      int res = send(sockfd, bytes + write_count, count - write_count, 0);
      if (res <= 0)
      {
        closeConnection();
        return SocketError::ERR_WRITE_FAIL;
      }

      write_count += res;
    }

    if (write_count != count)
    {
      return SocketError::ERR_INCOMPLETE_WRITE;
    }

    last_tx = esp_timer_get_time();
    return SocketError::OK;
  }

  bool isConnected() const
  {
    return connected;
  }

  bool operator()() const
  {
    return isConnected();
  }

  bool operator!() const
  {
    return !isConnected();
  }

  int64_t getIdleTime() const
  {
    return esp_timer_get_time() - last_tx;
  }

  ~SocketClient()
  {
    closeConnection();
    vSemaphoreDelete(mutex);
    mutex = nullptr;
  }
};
