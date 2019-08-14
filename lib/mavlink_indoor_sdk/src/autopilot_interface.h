// based on https://github.com/mavlink/c_uart_interface_example

#ifndef AUTOPILOT_INTERACE_H
#define AUTOPILOT_INTERACE_H

#include "helpers.h"
#include <pthread.h> // This uses POSIX Threads
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include "lowlevel.h"

#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION 0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY 0b0000110111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION 0b0000110000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE 0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE 0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE 0b0000010111111111
using namespace std;
namespace mavlink_indoor_sdk
{
namespace autopilot_interface
{
// uint64_t get_time_usec();
void set_position(float x, float y, float z, MAV_FRAME frame, mavlink_set_position_target_local_ned_t &sp);
void set_velocity(float vx, float vy, float vz, MAV_FRAME frame, mavlink_set_position_target_local_ned_t &sp);
void set_acceleration(float ax, float ay, float az, MAV_FRAME frame, mavlink_set_position_target_local_ned_t &sp);
void set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp);
void set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp);

// void set_VPE_data(float x, float y, float z, float roll, float pitch, float yaw, mavlink_vision_position_estimate_t &sp);
void *start_autopilot_interface_read_thread(void *args);
void *start_autopilot_interface_write_thread(void *args);
struct Time_Stamps
{
    Time_Stamps()
    {
        reset_timestamps();
    }

    uint64_t heartbeat;
    uint64_t sys_status;
    uint64_t battery_status;
    uint64_t radio_status;
    uint64_t local_position_ned;
    uint64_t global_position_int;
    uint64_t position_target_local_ned;
    uint64_t position_target_global_int;
    uint64_t highres_imu;
    uint64_t attitude;
    uint64_t local_position_vision_ned;

    void
    reset_timestamps()
    {
        heartbeat = 0;
        sys_status = 0;
        battery_status = 0;
        radio_status = 0;
        local_position_ned = 0;
        global_position_int = 0;
        position_target_local_ned = 0;
        position_target_global_int = 0;
        highres_imu = 0;
        attitude = 0;
        local_position_vision_ned = 0;
    }
};
struct Mavlink_Messages
{

    int sysid;
    int compid;

    // Heartbeat
    mavlink_heartbeat_t heartbeat;

    // System Status
    mavlink_sys_status_t sys_status;

    // Battery Status
    mavlink_battery_status_t battery_status;

    // Radio Status
    mavlink_radio_status_t radio_status;

    // Local Position
    mavlink_local_position_ned_t local_position_ned;
    mavlink_local_position_ned_cov_t local_position_vision_ned;

    // Global Position
    mavlink_global_position_int_t global_position_int;

    // Local Position Target
    mavlink_position_target_local_ned_t position_target_local_ned;
    // mavlin
    // mavlink_position
    // Global Position Target
    mavlink_position_target_global_int_t position_target_global_int;

    // HiRes IMU
    mavlink_highres_imu_t highres_imu;

    // Attitude
    mavlink_attitude_t attitude;

    // System Parameters?

    // Time Stamps
    Time_Stamps time_stamps;

    void
    reset_timestamps()
    {
        time_stamps.reset_timestamps();
    }
};

class AutopilotInterface
{
public:
    AutopilotInterface();
    AutopilotInterface(LowLevel *lowlevel_protocol_);
    int system_id;
    int autopilot_id;
    int companion_id;

    char reading_status;
    char writing_status;
    char control_status;

    void start();
    void stop();

    void enable_offboard_control();
    void disable_offboard_control();

    void update_setpoint(mavlink_set_position_target_local_ned_t setpoint);
    void update_nav_setpoint(mavlink_set_position_target_local_ned_t setpoint, float speed);

    void handle_quit(int sig);
    void start_read_thread();
    void start_write_thread(void);

    int write_message(mavlink_message_t message);
    void calc_nav_setpoint();
    Mavlink_Messages current_messages;
    bool connected = false;
    // void navigate()
private:
    
    bool time_to_exit =false;
    uint64_t write_count;

    
    mavlink_set_position_target_local_ned_t initial_position;
    mavlink_set_position_target_local_ned_t current_setpoint;

    // mavlink_set_position_target_local_ned_t target_setpoint;
    mavlink_set_position_target_local_ned_t nav_setpoint;
    mavlink_set_position_target_local_ned_t nav_start_setpoint;

    float nav_speed = 1;
    uint64_t nav_time_stamp = 1;
    bool with_nav_setpoint = false;
    float nav_distance = 0;

    pthread_t read_tid;
    pthread_t write_tid;
    void read_messages();
    void read_thread();
    void write_thread(void);

    void write_setpoint();

    int toggle_offboard_control(bool flag);
    LowLevel *lowlevel_protocol;
};
} // namespace autopilot_interface
} // namespace mavlink_indoor_sdk

#endif