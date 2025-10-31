#include <cstdint>
#include <vector>
#include <string>
#ifdef _WIN32
#include <winsock.h>
#else
#include <lwip/def.h>
#endif

using std::string;
using std::vector;

struct Heartbeat
{
	static const int32_t id = 1000;
	int8_t rssi;

	int32_t size() const
	{
		return 5;
	}

	vector<uint8_t> to_bytes() const
	{
		vector<uint8_t> bytes(size());

		const int32_t tmp_id = htonl(id);
		memcpy(bytes.data(), &tmp_id, 4);
		bytes[4] = (uint8_t)rssi;
		return bytes;
	}

	Heartbeat(int8_t rssi) : rssi(rssi)
	{
	}
};

struct TextMessage
{
	static const int32_t id = 1001;
	const string msg;
	const int32_t text_len;
	int8_t rssi;

	int32_t size() const
	{
		return 8 + text_len + 1;
	}

	vector<uint8_t> to_bytes() const
	{
		vector<uint8_t> bytes(size());

		const int32_t tmp_id = htonl(id);
		const int32_t tmp_length = htonl(text_len);
		memcpy(bytes.data(), &tmp_id, 4);
		memcpy(bytes.data() + 4, &tmp_length, 4);
		memcpy(bytes.data() + 8, msg.c_str(), text_len);
		bytes[8 + text_len] = rssi;
		return bytes;
	}

	TextMessage(const char* msg, int8_t rssi) : msg(msg), text_len((int)strlen(msg)), rssi(rssi)
	{
	}
};

struct Data
{
	static const int32_t id = 1002;
	int32_t vr;
	int32_t vl;

	int32_t size() const
	{
		return 12;
	}

	vector<uint8_t> to_bytes() const
	{
		vector<uint8_t> bytes(size());
		const int32_t tmp_id = htonl(id);
		const int32_t tmp_vr = htonl(vr);
		const int32_t tmp_vl = htonl(vl);
		memcpy(bytes.data(), &tmp_id, 4);
		memcpy(bytes.data() + 4, &tmp_vr, 4);
		memcpy(bytes.data() + 8, &tmp_vl, 4);
		return bytes;
	}

	Data() : vr(0), vl(0)
	{
	}

	Data(const int32_t vr, const int32_t vl) : vr(vr), vl(vl)
	{
	}
};

struct WhoAmI
{
	static const int32_t id = 1003;
	int32_t uid;

	int32_t size() const
	{
		return 8;
	}

	vector<uint8_t> to_bytes() const
	{
		vector<uint8_t> bytes(size());

		const int32_t tmp_id = htonl(id);
		const int32_t tmp_uid = htonl(uid);
		memcpy(bytes.data(), &tmp_id, 4);
		memcpy(bytes.data() + 4, &tmp_uid, 4);
		return bytes;
	}

	WhoAmI(const int32_t uid) : uid(uid)
	{
	}
};

const Data noDataC = Data();
