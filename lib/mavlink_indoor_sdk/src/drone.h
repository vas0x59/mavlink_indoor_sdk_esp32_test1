// by Vasily Yuryev

#ifndef DRONE_H
#define DRONE_H

#include "helpers.h"
#include "autopilot_interface.h"
// #include "lowlevel.h"

namespace mavlink_indoor_sdk
{
class Drone
{
public:
    // Drone();
    Drone(autopilot_interface::AutopilotInterface *ai_);
    ~Drone();

    Telemetry get_telemetry(Frame frame);
    void navigate(PointXYZyaw pose, Frame frame, float speed);
    void navigate_wait(PointXYZyaw pose, Frame frame, float speed, float thresh = 0.25);
    void set_position(PointXYZyaw pose, Frame frame);
    // void 
    // void set_velocity(PointXYZ, Frame frame);
    void arm();
    void disarm();
    
    void land();
    void takeoff(float alt, float speed);
    void sleep(uint16_t msec);
    void start();
    void stop();
    // void navigate()
private:
    autopilot_interface::AutopilotInterface *ai;
    void toggle_arming(bool arm_);
    // void mode_converter();
    // float nav_speed;

};

} // namespace mavlink_indoor_sdk

#endif