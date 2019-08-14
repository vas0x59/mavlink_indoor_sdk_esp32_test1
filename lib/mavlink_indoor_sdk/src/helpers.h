// by Vasily Yuryev

#ifndef HELPERS_H
#define HELPERS_H
#include <Arduino.h>
#include <iostream>
#include <vector>
#include <sys/time.h>
#include "mavlink_types.h"
#include "common/mavlink.h"
#include "mavlink_conversions.h"
#include "mavlink_helpers.h"
#include "px4_custom_mode.h"
#include <pthread.h> // This uses POSIX Threads
// #include <string>
#include "px4_custom_mode.h"

using namespace std;
namespace mavlink_indoor_sdk
{

uint64_t get_time_usec();

enum Mode
{
    MODE_MANUAL,
    MODE_ALTITUDE,
    MODE_POSITION,
    MODE_MISSION,
    MODE_HOLD,
    MODE_RETURN,
    MODE_ACRO,
    MODE_OFFBOARD,
    MODE_STABILIZE,
    MODE_RATTITUDE,
    MODE_TAKEOFF,
    MODE_LAND,
    MODE_FOLLOW_ME,
    MODE_UNASSIGNED
};


struct PointXYZ
{
    float x;
    float y;
    float z;
    // String to_String
    String ToString();
    // operator
};
/*
PointXYZ operator+(PointXYZ q, PointXYZ r)
{
    q.x = q.x + r.x;
    q.y = q.y + r.y;
    q.z = q.z + r.z;
    return q;
}
PointXYZ operator-(PointXYZ q, PointXYZ r)
{
    q.x = q.x - r.x;
    q.y = q.y - r.y;
    q.z = q.z - r.z;
    return q;
}
*/
struct RotationRPY
{
    float roll;
    float pitch;
    float yaw;

    String ToString();
};
struct Battery
{
    int cells_count;
    std::vector<float> cells_voltage;
    float voltage;

    String ToString();
};
struct PointXYZyaw
{
    float x;
    float y;
    float z;
    float yaw;

    String ToString();
};
struct Telemetry
{
    PointXYZ position;
    PointXYZ velocity;
    RotationRPY rotation;
    Mode mode;
    bool armed;
    bool connected;
    Battery battery;

    String ToString();
};
enum Frame
{
    FRAME_VISION,
    FRAME_LOCAL,
    FRAME_BODY
};

String frame_ToString(Frame frame);
String mode_ToString(Mode mode);
//functions
float get_dist(PointXYZ p1, PointXYZ p2);

MAV_FRAME frame_to_mav_frame(Frame frame);
Mode custom_mode_to_mode(uint32_t mav_mode);
Mode base_mode_to_mode(uint8_t mav_mode);
// bool base_mode_to_arm(uint8_t mav_mode);

PointXYZ enu_to_ned(PointXYZ enu);
PointXYZ ned_to_enu(PointXYZ ned);

PointXYZyaw enu_to_ned(PointXYZyaw enu);
PointXYZyaw ned_to_enu(PointXYZyaw ned);

} // namespace mavlink_indoor_sdk
#endif