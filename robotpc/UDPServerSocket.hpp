#include "UDPSocket.hpp"

class UDPServerSocket : public UDPSocket
{
private:
public:
    UDPServerSocket()
    {
    }

    bool create(const int port, const bool blocking = false)
    {
        if (createUDPServerSocket(port) != SockErr::ERR_OK)
        {
            return false;
        }
        if (setBlocking(blocking))
        {
            return false;
        }
        return true;
    }
};