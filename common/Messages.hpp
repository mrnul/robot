#pragma once

#include <cstdint>
#include <vector>
#include <optional>
#include <algorithm>
#include <span>
#ifdef _WIN32
#include <WinSock2.h>
#else
#include <lwip/def.h>
#endif

using std::array;
using std::copy_n;
using std::nullopt;
using std::optional;
using std::span;
using std::vector;

struct Heartbeat
{
	static constexpr uint32_t msg_size = 6;
	static constexpr uint32_t id = 1000;

	int8_t rssi;
	uint8_t uid;

	array<uint8_t, msg_size> toBytes() const noexcept
	{
		array<uint8_t, msg_size> bytes = {};
		const uint32_t tmp_id = htonl(id);
		memcpy(bytes.data(), &tmp_id, 4);
		bytes[4] = (uint8_t)rssi;
		bytes[5] = uid;
		return bytes;
	}

	static optional<Heartbeat> fromBuffer(const span<uint8_t> &buffer) noexcept
	{
		if (buffer.size() < msg_size)
			return nullopt;

		return Heartbeat((int8_t)buffer[4], buffer[5]);
	}

	Heartbeat(int8_t rssi, uint8_t uid) : rssi(rssi), uid(uid)
	{
	}
};

struct TextMessage
{
	static constexpr uint32_t msg_size = 38;
	static constexpr uint32_t id = 1001;
	char message[32];
	int8_t rssi;
	uint8_t uid;

	array<uint8_t, msg_size> toBytes() const noexcept
	{
		array<uint8_t, msg_size> bytes = {};

		const uint32_t tmp_id = htonl(id);
		memcpy(bytes.data(), &tmp_id, 4);
		memcpy(bytes.data() + 4, message, 32);
		bytes[36] = rssi;
		bytes[37] = uid;
		return bytes;
	}

	static optional<TextMessage> fromBuffer(const span<uint8_t> &buffer) noexcept
	{
		if (buffer.size() < msg_size)
			return nullopt;

		return TextMessage((char *)buffer.data() + 4, (int8_t)buffer[36], buffer[37]);
	}

	TextMessage(const char *msg, int8_t rssi, uint8_t uid) : message({}), rssi(rssi), uid(uid)
	{
		const size_t len = strlen(msg);
		copy_n(msg, len > 31 ? 31 : len, message);
		message[31] = 0;
	}
};

struct ControlData
{
	static constexpr uint32_t msg_size = 12;
	static constexpr uint32_t id = 1002;
	int32_t vr;
	int32_t vl;

	array<uint8_t, msg_size> toBytes() const noexcept
	{
		array<uint8_t, msg_size> bytes = {};

		const uint32_t tmp_id = htonl(id);
		const int32_t tmp_vr = htonl(vr);
		const int32_t tmp_vl = htonl(vl);
		memcpy(bytes.data(), &tmp_id, 4);
		memcpy(bytes.data() + 4, &tmp_vr, 4);
		memcpy(bytes.data() + 8, &tmp_vl, 4);

		return bytes;
	}

	static optional<ControlData> fromBuffer(const span<uint8_t> &buffer) noexcept
	{
		if (buffer.size() < msg_size)
			return nullopt;

		int32_t vr = 0;
		int32_t vl = 0;

		memcpy(&vr, buffer.data() + 4, 4);
		memcpy(&vl, buffer.data() + 8, 4);

		return ControlData(ntohl(vr), ntohl(vl));
	}

	ControlData(const int32_t vr, const int32_t vl) : vr(vr), vl(vl)
	{
	}
};

struct WhoAmI
{
	static constexpr uint32_t msg_size = 5;
	static constexpr uint32_t id = 1003;
	uint8_t uid;

	array<uint8_t, msg_size> toBytes() const noexcept
	{
		array<uint8_t, msg_size> bytes = {};

		const uint32_t tmp_id = htonl(id);
		memcpy(bytes.data(), &tmp_id, 4);

		bytes[4] = uid;

		return bytes;
	}

	static optional<WhoAmI> fromBuffer(const span<uint8_t> &buffer) noexcept
	{
		if (buffer.size() < msg_size)
			return nullopt;

		return WhoAmI(buffer[4]);
	}

	WhoAmI(const uint8_t uid) : uid(uid)
	{
	}
};

enum ColorOrder : uint8_t
{
	RGB = 0,
	GRB = 1
};

struct LEDData
{
	static constexpr uint32_t msg_size = 12;
	static constexpr uint32_t id = 1004;

	uint32_t gpio_num;
	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t colorOrder;

	array<uint8_t, msg_size> toBytes() const noexcept
	{
		array<uint8_t, msg_size> bytes = {};

		const uint32_t tmp_id = htonl(id);
		const uint32_t tmp_gpio_num = htonl(gpio_num);
		memcpy(bytes.data(), &tmp_id, 4);
		memcpy(bytes.data() + 4, &tmp_gpio_num, 4);

		bytes[8] = r;
		bytes[9] = g;
		bytes[10] = b;
		bytes[11] = colorOrder;

		return bytes;
	}

	static optional<LEDData> fromBuffer(const span<uint8_t> &buffer) noexcept
	{
		if (buffer.size() < msg_size)
			return nullopt;

		uint32_t gpio_num = 0;
		memcpy(&gpio_num, buffer.data() + 4, 4);

		return LEDData(ntohl(gpio_num), buffer[8], buffer[9], buffer[10], buffer[11]);
	}

	LEDData(uint32_t gpio_num, uint8_t r, uint8_t g, uint8_t b, uint8_t colorOrder) : gpio_num(gpio_num), r(r), g(g), b(b), colorOrder(colorOrder)
	{
	}
};

struct RequestWhoAmI
{
	static constexpr uint32_t msg_size = 4;
	static constexpr uint32_t id = 1005;

	array<uint8_t, msg_size> toBytes() const noexcept
	{
		array<uint8_t, msg_size> bytes = {};

		const uint32_t tmp_id = htonl(id);
		memcpy(bytes.data(), &tmp_id, 4);
		return bytes;
	}

	static optional<RequestWhoAmI> fromBuffer(const span<uint8_t> &buffer) noexcept
	{
		if (buffer.size() < msg_size)
			return nullopt;

		return RequestWhoAmI();
	}

	RequestWhoAmI()
	{
	}
};
