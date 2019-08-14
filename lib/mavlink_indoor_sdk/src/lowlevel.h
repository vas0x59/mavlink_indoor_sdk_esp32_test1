// #include "helpers.h"

#ifndef LOWLEVEL_H
#define LOWLEVEL_H

#include "mavlink_types.h"
#include "common/mavlink.h"
#include "mavlink_conversions.h"
#include "mavlink_helpers.h"
#include "px4_custom_mode.h"
#include <string>

using namespace std;

namespace mavlink_indoor_sdk
{
class LowLevel
{
public:
    // LowLevel(string url){};
    virtual void start(){};
    virtual void stop(){};

    virtual int read_message(mavlink_message_t &message){return 0;};
    virtual int write_message(mavlink_message_t &message){return 0;};
    int status = 1;
    // void start(){};
    // void stop(){};
};
};

#endif