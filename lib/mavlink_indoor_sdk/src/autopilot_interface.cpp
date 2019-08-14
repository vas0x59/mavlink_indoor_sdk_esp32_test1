// based on https://github.com/mavlink/c_uart_interface_example
#include "autopilot_interface.h"
#include <pthread.h>
#include <stdio.h>

#include <signal.h>
using namespace std;

namespace mavlink_indoor_sdk
{
namespace autopilot_interface
{

void set_position(float x, float y, float z, MAV_FRAME frame, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask =
        MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;

    sp.coordinate_frame = frame;

    sp.x = x;
    sp.y = y;
    sp.z = z;

    // printf("POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);
}

/*
 * Set target local ned velocity
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target VX VY VZ
 * velocities in the Local NED frame, in meters per second.
 */
void set_velocity(float vx, float vy, float vz, MAV_FRAME frame, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask =
        MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY;

    sp.coordinate_frame = frame;

    sp.vx = vx;
    sp.vy = vy;
    sp.vz = vz;

    //printf("VELOCITY SETPOINT UVW = [ %.4f , %.4f , %.4f ] \n", sp.vx, sp.vy, sp.vz);
}

/*
 * Set target local ned acceleration
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target AX AY AZ
 * accelerations in the Local NED frame, in meters per second squared.
 */
void set_acceleration(float ax, float ay, float az, MAV_FRAME frame, mavlink_set_position_target_local_ned_t &sp)
{

    // NOT IMPLEMENTED
    // fprintf(stderr, "set_acceleration doesn't work yet \n");
    throw 1;

    sp.type_mask =
        MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION &
        MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY;

    sp.coordinate_frame = frame;
    // sp./
    sp.afx = ax;
    sp.afy = ay;
    sp.afz = az;
}

// the next two need to be called after one of the above

/*
 * Set target local ned yaw
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw
 * in the Local NED frame, in radians.
 */
void set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask &=
        MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;

    sp.yaw = yaw;

    printf("POSITION SETPOINT YAW = %.4f \n", sp.yaw);
}

/*
 * Set target local ned yaw rate
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw rate
 * in the Local NED frame, in radians per second.
 */
void set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask &=
        MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;

    sp.yaw_rate = yaw_rate;
}

AutopilotInterface::AutopilotInterface()
{
    write_count = 0;

    reading_status = 0;   // whether the read thread is running
    writing_status = 0;   // whether the write thread is running
    control_status = 0;   // whether the autopilot is in offboard control mode
    time_to_exit = false; // flag to signal thread exit

    read_tid = 0;  // read thread id
    write_tid = 0; // write thread id

    system_id = 0;    // system id
    autopilot_id = 0; // autopilot component id
    companion_id = 0; // companion computer component id

    current_messages.sysid = system_id;
    current_messages.compid = autopilot_id;
}

AutopilotInterface::AutopilotInterface(LowLevel *lowlevel_protocol_)
{
    write_count = 0;

    reading_status = 0;   // whether the read thread is running
    writing_status = 0;   // whether the write thread is running
    control_status = 0;   // whether the autopilot is in offboard control mode
    time_to_exit = false; // flag to signal thread exit

    read_tid = 0;  // read thread id
    write_tid = 0; // write thread id

    system_id = 0;    // system id
    autopilot_id = 0; // autopilot component id
    companion_id = 0; // companion computer component id

    current_messages.sysid = system_id;
    current_messages.compid = autopilot_id;

    lowlevel_protocol = lowlevel_protocol_;
}

void AutopilotInterface::update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
    current_setpoint = setpoint;
}
void AutopilotInterface::update_nav_setpoint(mavlink_set_position_target_local_ned_t t_setpoint, float speed)
{
    nav_setpoint = t_setpoint;
    nav_speed = speed;
    with_nav_setpoint = true;
    nav_time_stamp = (uint32_t)(get_time_usec() / 1000);
    nav_start_setpoint = current_setpoint;
    nav_distance = get_dist({nav_start_setpoint.x, nav_start_setpoint.y, nav_start_setpoint.z},
                            {nav_setpoint.x, nav_setpoint.y, nav_setpoint.z});
}

void AutopilotInterface::read_messages()
{
    bool success;              // receive success flag
    bool received_all = false; // receive only one message
    Time_Stamps this_timestamps;

    // Blocking wait for new data
    while (!received_all and !time_to_exit)
    // while (true)
    {
        // ----------------------------------------------------------------------
        //   READ MESSAGE
        // ----------------------------------------------------------------------
        mavlink_message_t message;
        success = lowlevel_protocol->read_message(message);

        // ----------------------------------------------------------------------
        //   HANDLE MESSAGE
        // ----------------------------------------------------------------------
        if (success)
        {

            // Store message sysid and compid.
            // Note this doesn't handle multiple message sources.
            current_messages.sysid = message.sysid;
            current_messages.compid = message.compid;

            // Handle Message ID
            switch (message.msgid)
            {

            case MAVLINK_MSG_ID_HEARTBEAT:
            {
                //printf("MAVLINK_MSG_ID_HEARTBEAT\n");
                mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
                current_messages.time_stamps.heartbeat = get_time_usec();
                this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
                break;
            }

            case MAVLINK_MSG_ID_SYS_STATUS:
            {
                //printf("MAVLINK_MSG_ID_SYS_STATUS\n");
                mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
                current_messages.time_stamps.sys_status = get_time_usec();
                this_timestamps.sys_status = current_messages.time_stamps.sys_status;
                break;
            }

            case MAVLINK_MSG_ID_BATTERY_STATUS:
            {
                //printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
                mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
                current_messages.time_stamps.battery_status = get_time_usec();
                this_timestamps.battery_status = current_messages.time_stamps.battery_status;
                break;
            }

            case MAVLINK_MSG_ID_RADIO_STATUS:
            {
                //printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
                mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
                current_messages.time_stamps.radio_status = get_time_usec();
                this_timestamps.radio_status = current_messages.time_stamps.radio_status;
                break;
            }

            case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
            {
                //printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");

                mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
                current_messages.time_stamps.local_position_ned = get_time_usec();
                this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
                break;
            }
            case MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV:
            {
                mavlink_local_position_ned_cov_t mss;
                mavlink_msg_local_position_ned_cov_decode(&message, &mss);

                if (mss.estimator_type == MAV_ESTIMATOR_TYPE_VISION)
                {
                    current_messages.local_position_vision_ned = mss;
                    current_messages.time_stamps.local_position_vision_ned = get_time_usec();
                    this_timestamps.local_position_vision_ned = current_messages.time_stamps.local_position_vision_ned;
                }
                // if (mss.)

                // current_messages.local_position_vision_ned.
            }

            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            {
                //printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
                mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
                current_messages.time_stamps.global_position_int = get_time_usec();
                this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
                break;
            }

            case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
            {
                //printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
                mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
                current_messages.time_stamps.position_target_local_ned = get_time_usec();
                this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
                break;
            }

            case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
            {
                //printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
                mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
                current_messages.time_stamps.position_target_global_int = get_time_usec();
                this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
                break;
            }

            case MAVLINK_MSG_ID_HIGHRES_IMU:
            {
                //printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
                mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
                current_messages.time_stamps.highres_imu = get_time_usec();
                this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
                break;
            }

            case MAVLINK_MSG_ID_ATTITUDE:
            {
                //printf("MAVLINK_MSG_ID_ATTITUDE\n");
                mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
                current_messages.time_stamps.attitude = get_time_usec();
                this_timestamps.attitude = current_messages.time_stamps.attitude;
                break;
            }

            default:
            {
                // printf("Warning, did not handle message id %i\n",message.msgid);
                break;
            }

            } // end: switch msgid

        } // end: if read message

        // Check for receipt of all items
        received_all =
            this_timestamps.heartbeat &&
            this_timestamps.battery_status &&
            this_timestamps.radio_status &&
            this_timestamps.local_position_ned &&
            this_timestamps.local_position_vision_ned &&
            this_timestamps.global_position_int &&
            this_timestamps.position_target_local_ned &&
            this_timestamps.position_target_global_int &&
            this_timestamps.highres_imu &&
            this_timestamps.attitude &&
            this_timestamps.sys_status;

        if ((get_time_usec() - this_timestamps.heartbeat) < 2*1000*1000){
            connected = true;
        }
        else{
            connected = false;
        }
        // give the write thread time to use the port
        if (writing_status > false)
        {
            usleep(100); // look for components of batches at 10kHz
        }

    } // end: while not received all

    return;
}

int AutopilotInterface::toggle_offboard_control(bool flag)
{
    // Prepare command for off-board mode
    // mavlink_command_long_t com = {0};
    // com.target_system = system_id;
    // com.target_component = MAV_COMP_ID_ALL;
    // com.command = MAV_CMD_NAV_GUIDED_ENABLE;
    // com.confirmation = true;
    // com.param1 = (float)flag; // flag >0.5 => start, <0.5 => stop

    // // Encode
    // mavlink_message_t message;
    // mavlink_msg_command_long_encode(system_id, MAV_COMP_ID_ALL, &message, &com);

    // Send the message
    // int len = lowlevel_protocol->write_message(message);

    mavlink_message_t msg;
    mavlink_command_long_t cmd = {};

    cmd.command = MAV_CMD_DO_SET_MODE;
    cmd.target_system = 1;
    cmd.target_component = MAV_COMP_ID_ALL;

    cmd.param1 = 1;
    if (flag)
        cmd.param2 = 6;
    else
        cmd.param2 = 12;

    mavlink_msg_command_long_encode(system_id, MAV_COMP_ID_ALL, &msg, &cmd);
    int len = lowlevel_protocol->write_message(msg);

    // Done!
    return len;
}

void AutopilotInterface::enable_offboard_control()
{
    // Should only send this command once
    if (control_status == false)
    {
        printf("ENABLE OFFBOARD MODE\n");

        int success = toggle_offboard_control(true);

        if (success)
            control_status = true;
        else
        {
            // fprintf(stderr, "Error: off-board mode not set, could not write message\n");
            //throw EXIT_FAILURE;
        }

        printf("\n");

    } // end: if not offboard_status
}

// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
void AutopilotInterface::disable_offboard_control()
{

    if (control_status == true)
    {
        printf("DISABLE OFFBOARD MODE\n");

        int success = toggle_offboard_control(false);

        if (success)
            control_status = false;
        else
        {
            // fprintf(stderr, "Error: off-board mode not set, could not write message\n");
            //throw EXIT_FAILURE;
        }

        printf("\n");

    } // end: if offboard_status
}
void AutopilotInterface::start()
{
    int result;

    // --------------------------------------------------------------------------
    //   CHECK SERIAL PORT
    // --------------------------------------------------------------------------

    if (lowlevel_protocol->status != 1) // SERIAL_PORT_OPEN
    {
        // fprintf(stderr, "ERROR: serial port not open\n");
        throw 1;
    }

    // --------------------------------------------------------------------------
    //   READ THREAD
    // --------------------------------------------------------------------------

    printf("START READ THREAD \n");

    result = pthread_create(&read_tid, NULL, &start_autopilot_interface_read_thread, this);
    if (result)
        throw result;

    // now we're reading messages
    printf("\n");

    // --------------------------------------------------------------------------
    //   CHECK FOR MESSAGES
    // --------------------------------------------------------------------------
    mavlink_message_t msg;
    // mavlink
    // MAV_TYPE_ONBOARD_CONTROLLER
    mavlink_heartbeat_t hb;
    hb.type = MAV_TYPE_ONBOARD_CONTROLLER;
    // hb.system_status = 1;
    // hb.
    mavlink_msg_heartbeat_encode(2, 0, &msg, &hb);
    // lowlevel_protocol->write_message(msg);
    printf("CHECK FOR MESSAGES\n");

    while (not current_messages.sysid)
    {
        if (time_to_exit)
            return;
        usleep(500000); // check at 2Hz
    }

    printf("Found\n");

    // now we know autopilot is sending messages
    printf("\n");

    // --------------------------------------------------------------------------
    //   GET SYSTEM and COMPONENT IDs
    // --------------------------------------------------------------------------

    // This comes from the heartbeat, which in theory should only come from
    // the autopilot we're directly connected to it.  If there is more than one
    // vehicle then we can't expect to discover id's like this.
    // In which case set the id's manually.

    // System ID
    if (not system_id)
    {
        system_id = 1;
        printf("GOT VEHICLE SYSTEM ID: %i\n", system_id);
    }

    // Component ID
    if (not autopilot_id)
    {
        autopilot_id = current_messages.compid;
        printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
        printf("\n");
    }

    // --------------------------------------------------------------------------
    //   GET INITIAL POSITION
    // --------------------------------------------------------------------------

    // Wait for initial position ned
    while (not(current_messages.time_stamps.local_position_ned &&
               current_messages.time_stamps.attitude))
    {
        if (time_to_exit)
            return;

        usleep(500000);
    }

    // copy initial position ned
    Mavlink_Messages local_data = current_messages;
    initial_position.x = local_data.local_position_ned.x;
    initial_position.y = local_data.local_position_ned.y;
    initial_position.z = local_data.local_position_ned.z;
    initial_position.vx = local_data.local_position_ned.vx;
    initial_position.vy = local_data.local_position_ned.vy;
    initial_position.vz = local_data.local_position_ned.vz;
    initial_position.yaw = local_data.attitude.yaw;
    initial_position.yaw_rate = local_data.attitude.yawspeed;

    printf("INITIAL POSITION XYZ = [ %.4f , %.4f , %.4f ] \n", initial_position.x, initial_position.y, initial_position.z);
    printf("INITIAL POSITION YAW = %.4f \n", initial_position.yaw);
    printf("\n");

    // we need this before starting the write thread

    // --------------------------------------------------------------------------
    //   WRITE THREAD
    // --------------------------------------------------------------------------
    printf("START WRITE THREAD \n");

    result = pthread_create(&write_tid, NULL, &start_autopilot_interface_write_thread, this);
    if (result)
        throw result;

    // wait for it to be started
    while (not writing_status)
        usleep(100000); // 10Hz

    // now we're streaming setpoint commands
    printf("\n");

    // Done!
    return;
}

void AutopilotInterface::start_read_thread()
{

    if (reading_status != 0)
    {
        // fprintf(stderr, "read thread already running\n");
        return;
    }
    else
    {
        read_thread();
        return;
    }
}

// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void AutopilotInterface::start_write_thread(void)
{
    if (not writing_status == false)
    {
        // fprintf(stderr, "write thread already running\n");
        return;
    }

    else
    {
        write_thread();
        return;
    }
}

void AutopilotInterface::stop()
{

    printf("CLOSE THREADS\n");

    // signal exit
    time_to_exit = true;

    // wait for exit
    pthread_join(read_tid, NULL);
    pthread_join(write_tid, NULL);

    // now the read and write threads are closed
    printf("\n");

    // still need to close the serial_port separately
}
void AutopilotInterface::
    handle_quit(int sig)
{

    disable_offboard_control();

    try
    {
        stop();
    }
    catch (int error)
    {
        // fprintf(stderr, "Warning, could not stop autopilot interface\n");
    }
}

void AutopilotInterface::calc_nav_setpoint()
{
    if (with_nav_setpoint == true)
    {

        float stamp = (uint32_t)(get_time_usec() / 1000);
        float time = nav_distance / nav_speed;
        // float distance = get_dist(nav_start.pose.position, setpoint_position_transformed.pose.position);
        // float time = distance / speed;
        float passed = ((float)(stamp - nav_time_stamp) / 1000.0f) / time;
        
        if (passed >= 1){
            with_nav_setpoint = false;
        }
        current_setpoint.x = nav_start_setpoint.x + (nav_setpoint.x - nav_start_setpoint.x) * passed;
        current_setpoint.y = nav_start_setpoint.y + (nav_setpoint.y - nav_start_setpoint.y) * passed;
        current_setpoint.z = nav_start_setpoint.z + (nav_setpoint.z - nav_start_setpoint.z) * passed;
        current_setpoint.yaw = nav_start_setpoint.yaw + (nav_setpoint.yaw - nav_start_setpoint.yaw) * passed;
        // PointXYZ p = {current_setpoint.x, current_setpoint.y, current_setpoint.z};

        // cout << "passed: " << passed << p.ToString() << "\n";
    }
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void AutopilotInterface::read_thread()
{
    reading_status = true;

    while (!time_to_exit)
    {
        read_messages();
        usleep(100000); // Read batches at 10Hz
    }

    reading_status = false;

    return;
}

int AutopilotInterface::
    write_message(mavlink_message_t message)
{
    // do the write
    int len = lowlevel_protocol->write_message(message);

    // book keep
    write_count++;

    // Done!
    return len;
}

void AutopilotInterface::write_setpoint()
{
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------

    // pull from position target
    mavlink_set_position_target_local_ned_t sp = current_setpoint;

    // double check some system parameters
    if (not sp.time_boot_ms)
        sp.time_boot_ms = (uint32_t)(get_time_usec() / 1000);
    sp.target_system = 1;
    sp.target_component = MAV_COMP_ID_ALL;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------

    mavlink_message_t message;
    mavlink_msg_set_position_target_local_ned_encode(system_id , MAV_COMP_ID_ALL, &message, &sp);
    // mavlink_msg_set_position_target_local_ned
    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    int len = write_message(message);

    // check the write
    // if (len <= 0)
        // fprintf(stderr, "WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
    //	else
    //		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

    // return;
}

// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void AutopilotInterface::write_thread(void)
{
    // signal startup
    writing_status = 2;

    // prepare an initial setpoint, just stay put
    mavlink_set_position_target_local_ned_t sp;
    sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
                   MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
    sp.vx = 0.0;
    sp.vy = 0.0;
    sp.vz = 0.0;
    sp.yaw_rate = 0.0;

    // set position target
    current_setpoint = sp;

    // write a message and signal writing
    write_setpoint();
    writing_status = true;

    // Pixhawk needs to see off-board commands at minimum 2Hz,
    // otherwise it will go into fail safe
    while (!time_to_exit)
    {
        usleep(100000); // Stream at 10Hz
        calc_nav_setpoint();
        write_setpoint();
    }

    // signal end
    writing_status = false;

    // return;
}

// End Autopilot_Interface

// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void *start_autopilot_interface_read_thread(void *args)
{
    // takes an autopilot object argument
    mavlink_indoor_sdk::autopilot_interface::AutopilotInterface *autopilot_interface =
        (mavlink_indoor_sdk::autopilot_interface::AutopilotInterface *)args;

    // run the object's read thread
    autopilot_interface->start_read_thread();

    // done!
    return NULL;
}

void *start_autopilot_interface_write_thread(void *args)
{
    // takes an autopilot object argument
    mavlink_indoor_sdk::autopilot_interface::AutopilotInterface *autopilot_interface =
        (mavlink_indoor_sdk::autopilot_interface::AutopilotInterface *)args;

    // run the object's read thread
    autopilot_interface->start_write_thread();

    // done!
    return NULL;
}

}; // namespace autopilot_interface
}; // namespace mavlink_indoor_sdk