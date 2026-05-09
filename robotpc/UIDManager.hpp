#pragma once

#include <vector>
#include <optional>
#include "Config.hpp"

using std::optional;
using std::vector;

class UIDManager
{
private:
    vector<bool> uids; // if uid is taken then uids[uid] = true

public:
    UIDManager()
        : uids(Config::maxRobotCount(), false)
    {
    }

    optional<uint8_t> getFirstAvailable()
    {
        for (uint8_t i = 0; i < uids.size(); i++)
        {
            if (!uids[i])
            {
                uids[i] = true;
                return i;
            }
        }
        return std::nullopt;
    }

    void releaseUID(const int uid)
    {
        uids[uid] = false;
    }
};