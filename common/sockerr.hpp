#pragma once

namespace sockerr
{
	enum SockErr
	{
		ERR_OK = 0,
		ERR_WSA = -1,
		ERR_CREATE = -2,
		ERR_BIND = -3,
		ERR_ALREADY_INIT = -4,
		ERR_CONNECT = -5,
		ERR_SEND = -6,
		ERR_PARTIAL_SEND = -7,
		ERR_INVALID_ADDR = -8,
	};
}