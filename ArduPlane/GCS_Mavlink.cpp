// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

// default sensors are present and healthy: gyro, accelerometer, barometer, rate_control, attitude_stabilization, yaw_position, altitude control, x/y position control, motor_control
#define MAVLINK_SENSOR_PRESENT_DEFAULT (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE | MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL | MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION | MAV_SYS_STATUS_SENSOR_YAW_POSITION | MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL | MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL | MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS | MAV_SYS_STATUS_AHRS | MAV_SYS_STATUS_SENSOR_RC_RECEIVER)

void Plane::send_heartbeat(mavlink_channel_t chan)
{
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status;
    uint32_t custom_mode = control_mode;

    mavlink_msg_heartbeat_send(
        chan,
        MAV_TYPE_FIXED_WING,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        custom_mode,
        system_status);
}

void Plane::send_debugtext(mavlink_channel_t chan)
{
	/////////////////////////////////////////////////////////
	///// WRITE DEBUG MESSAGES HERE

//    gcs_send_text_fmt(PSTR("compass heading %.1f deg"), (float)ahrs.yaw_sensor*0.01);
	  plane.gcs_send_text_fmt(MAV_SEVERITY_INFO, " %.3f, : %.3f,  %.3f", debug_dummy1, debug_dummy2,debug_dummy3);
	/////////////////////////////////////////////////////////
}

void Plane::send_pixhawk_hg_fast(mavlink_channel_t chan)
{
    Vector3f accel = ahrs.get_accel_ef_blended();
    mavlink_msg_pixhawk_hg_fast_send(
    	chan,
    	(uint32_t)((gps.time_epoch_usec() % 86400000000) / 1000),
        accel.x,
        accel.y,
        accel.z,
        ahrs.roll,
        ahrs.pitch,
        (uint16_t)ahrs.yaw_sensor);
}

void Plane::send_pixhawk_hg_med(mavlink_channel_t chan)
{
    Vector3f v;
    static float ias, tas;
    static uint32_t as_last_update=0;

    // check for new airspeed update
    uint32_t t = airspeed.last_update_ms();
    if( t != as_last_update){
		ias = airspeed.get_airspeed();
		tas = get_true_airspeed();
		as_last_update = t;
    }
    if( ! ahrs.get_velocity_NED(v)){
    	v.zero();
    }

    mavlink_msg_pixhawk_hg_med_send(
		chan,
		current_loc.lat,                 // in 1E7 degrees
		current_loc.lng,                 // in 1E7 degrees
		current_loc.alt * 10,            // millimeters above sea level
		(int16_t)(v.x * 100.),           // X speed cm/s (+ve North)
		(int16_t)(v.y * 100.),           // Y speed cm/s (+ve East)
		(int16_t)(v.z * 100.),           // Z speed cm/s (+ve up)
		(int16_t)(vario_TE * 100.),      // total energy vario cm/s (+ve up)
		ias,                             // indicated airspeed
		tas);                            // true airspeed
}

void Plane::send_pixhawk_hg_slow(mavlink_channel_t chan)
{
	Location loc = gps.location();
    Vector3f wind = ahrs.wind_estimate();
    // get temperature from airspeed sensor if available
	float temperature;
    if( ! airspeed.get_temperature(temperature)){
    	temperature = barometer.get_temperature();
    }

    mavlink_msg_pixhawk_hg_slow_send(
    	chan,
    	gps.time_epoch_usec(),
    	gps.get_hdop(),
    	loc.alt * 10, // in mm
    	gps.status(),
    	gps.num_sats(),
        temperature,
        0,  // humidity
        wind.x,  // North +ve
        wind.y,  // East +ve
        0); // Down +ve (not implemented)

}

void Plane::send_attitude(mavlink_channel_t chan){}
#if GEOFENCE_ENABLED == ENABLED
void Plane::send_fence_status(mavlink_channel_t chan){}
#endif
void Plane::send_extended_status1(mavlink_channel_t chan){}
void Plane::send_location(mavlink_channel_t chan){}
void Plane::send_nav_controller_output(mavlink_channel_t chan){}
void Plane::send_servo_out(mavlink_channel_t chan){}
void Plane::send_radio_out(mavlink_channel_t chan){}
void Plane::send_vfr_hud(mavlink_channel_t chan){}
#if HIL_SUPPORT
static mavlink_hil_state_t last_hil_state;
#endif
void Plane::send_simstate(mavlink_channel_t chan){}
void Plane::send_hwstatus(mavlink_channel_t chan){}
void Plane::send_wind(mavlink_channel_t chan){}
void NOINLINE Plane::send_rpm(mavlink_channel_t chan){}
void Plane::send_pid_tuning(mavlink_channel_t chan){}
void Plane::send_rangefinder(mavlink_channel_t chan){}
void Plane::send_current_waypoint(mavlink_channel_t chan){}

void Plane::send_statustext(mavlink_channel_t chan)
{
    mavlink_statustext_t *s = &gcs[chan-MAVLINK_COMM_0].pending_status;
    mavlink_msg_statustext_send(
        chan,
        s->severity,
        s->text);
}

// are we still delaying telemetry to try to avoid Xbee bricking?
bool Plane::telemetry_delayed(mavlink_channel_t chan)
{
    uint32_t tnow = millis() >> 10;
    if (tnow > (uint32_t)g.telem_delay) {
        return false;
    }
    if (chan == MAVLINK_COMM_0 && hal.gpio->usb_connected()) {
        // this is USB telemetry, so won't be an Xbee
        return false;
    }
    // we're either on the 2nd UART, or no USB cable is connected
    // we need to delay telemetry by the TELEM_DELAY time
    return true;
}


// try to send a message, return false if it won't fit in the serial tx buffer
bool GCS_MAVLINK::try_send_message(enum ap_message id)
{
    if (plane.telemetry_delayed(chan)) {
        return false;
    }

    // if we don't have at least 1ms remaining before the main loop
    // wants to fire then don't send a mavlink message. We want to
    // prioritise the main flight control loop over communications
    if (!plane.in_mavlink_delay && plane.scheduler.time_available_usec() < 1200) {
        plane.gcs_out_of_time = true;
        return false;
    }

    switch (id) {
    case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        plane.gcs[chan-MAVLINK_COMM_0].last_heartbeat_time = AP_HAL::millis();
        plane.send_heartbeat(chan);
        return true;

    case MSG_DEBUGTEXT:
        CHECK_PAYLOAD_SIZE(STATUSTEXT);
        plane.send_debugtext(chan);
        return true;

    case MSG_PIXHAWK_HG_FAST:
        CHECK_PAYLOAD_SIZE(PIXHAWK_HG_FAST);
        plane.send_pixhawk_hg_fast(chan);
        break;

    case MSG_PIXHAWK_HG_MED:
        CHECK_PAYLOAD_SIZE(PIXHAWK_HG_MED);
        plane.send_pixhawk_hg_med(chan);
        break;

    case MSG_PIXHAWK_HG_SLOW:
        CHECK_PAYLOAD_SIZE(PIXHAWK_HG_SLOW);
        plane.send_pixhawk_hg_slow(chan);
        break;
    }
    return true;
}


/*
  default stream rates to 1Hz
 */
const AP_Param::GroupInfo GCS_MAVLINK::var_info[] = {
    // @Param: RAW_SENS
    // @DisplayName: Raw sensor stream rate
    // @Description: Raw sensor stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK, streamRates[0],  25),

    // @Param: EXT_STAT
    // @DisplayName: Extended status stream rate to ground station
    // @Description: Extended status stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK, streamRates[1],  10),

	// @Param: POSITION
    // @DisplayName: Position stream rate to ground station
    // @Description: Position stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK, streamRates[4],  2),

	AP_GROUPEND
};


// see if we should send a stream now. Called at 50Hz
bool GCS_MAVLINK::stream_trigger(enum streams stream_num)
{
    if (stream_num >= NUM_STREAMS) {
        return false;
    }
    float rate = (uint8_t)streamRates[stream_num].get();

    // send at a much lower rate while handling waypoints and
    // parameter sends
    if ((stream_num != STREAM_PARAMS) && 
        (waypoint_receiving || _queued_parameter != NULL)) {
        rate *= 0.25f;
    }

    if (rate <= 0) {
        return false;
    }

    if (stream_ticks[stream_num] == 0) {
        // we're triggering now, setup the next trigger point
        if (rate > 50) {
            rate = 50;
        }
        stream_ticks[stream_num] = (50 / rate) - 1 + stream_slowdown;
        return true;
    }

    // count down at 50Hz
    stream_ticks[stream_num]--;
    return false;
}

void
GCS_MAVLINK::data_stream_send(void)
{
    plane.gcs_out_of_time = false;

    if (!plane.in_mavlink_delay) {
        handle_log_send(plane.DataFlash);
    }

    if (_queued_parameter != NULL) {
        if (streamRates[STREAM_PARAMS].get() <= 0) {
            streamRates[STREAM_PARAMS].set(10);
        }
        if (stream_trigger(STREAM_PARAMS)) {
            send_message(MSG_NEXT_PARAM);
        }
    }

    if (plane.gcs_out_of_time) return;

    if (plane.in_mavlink_delay) {
        // don't send any other stream types while in the delay callback
        return;
    }

    if (plane.gcs_out_of_time) return;

    if (stream_trigger(STREAM_RAW_SENSORS)) {
        send_message(MSG_PIXHAWK_HG_FAST);
    }

    if (plane.gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTENDED_STATUS)) {
        send_message(MSG_PIXHAWK_HG_MED);
        send_message(MSG_DEBUGTEXT);
    }

    if (plane.gcs_out_of_time) return;

    if (stream_trigger(STREAM_POSITION)) {
        send_message(MSG_PIXHAWK_HG_SLOW);
    }

}


/*
  handle a request to switch to guided mode. This happens via a
  callback from handle_mission_item()
 */
void GCS_MAVLINK::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
}

/*
  handle a request to change current WP altitude. This happens via a
  callback from handle_mission_item()
 */
void GCS_MAVLINK::handle_change_alt_request(AP_Mission::Mission_Command &cmd)
{
}

void GCS_MAVLINK::handleMessage(mavlink_message_t* msg)
{
    switch (msg->msgid) {

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
    {
        handle_request_data_stream(msg, true);
        break;
    }

    case MAVLINK_MSG_ID_COMMAND_LONG:
    {
        // decode
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(msg, &packet);

        uint8_t result = MAV_RESULT_UNSUPPORTED;

        // do command
        send_text(MAV_SEVERITY_INFO,"Command received: ");

        switch(packet.command) {

        case MAV_CMD_START_RX_PAIR:
            // initiate bind procedure
            if (!hal.rcin->rc_bind(packet.param1)) {
                result = MAV_RESULT_FAILED;
            } else {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_NAV_LOITER_UNLIM:
            plane.set_mode(LOITER);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
            plane.set_mode(RTL);
            result = MAV_RESULT_ACCEPTED;
            break;

#if MOUNT == ENABLED
        // Sets the region of interest (ROI) for the camera
        case MAV_CMD_DO_SET_ROI:
            // sanity check location
            if (fabsf(packet.param5) > 90.0f || fabsf(packet.param6) > 180.0f) {
                break;
            }
            Location roi_loc;
            roi_loc.lat = (int32_t)(packet.param5 * 1.0e7f);
            roi_loc.lng = (int32_t)(packet.param6 * 1.0e7f);
            roi_loc.alt = (int32_t)(packet.param7 * 100.0f);
            if (roi_loc.lat == 0 && roi_loc.lng == 0 && roi_loc.alt == 0) {
                // switch off the camera tracking if enabled
                if (plane.camera_mount.get_mode() == MAV_MOUNT_MODE_GPS_POINT) {
                    plane.camera_mount.set_mode_to_default();
                }
            } else {
                // send the command to the camera mount
                plane.camera_mount.set_roi_target(roi_loc);
            }
            result = MAV_RESULT_ACCEPTED;
            break;
#endif

#if CAMERA == ENABLED
        case MAV_CMD_DO_DIGICAM_CONFIGURE:
            plane.camera.configure(packet.param1,
                                   packet.param2,
                                   packet.param3,
                                   packet.param4,
                                   packet.param5,
                                   packet.param6,
                                   packet.param7);

            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_DO_DIGICAM_CONTROL:
            plane.camera.control(packet.param1,
                                 packet.param2,
                                 packet.param3,
                                 packet.param4,
                                 packet.param5,
                                 packet.param6);

            result = MAV_RESULT_ACCEPTED;
            break;
#endif // CAMERA == ENABLED

        case MAV_CMD_DO_MOUNT_CONTROL:
#if MOUNT == ENABLED
            plane.camera_mount.control(packet.param1, packet.param2, packet.param3, (MAV_MOUNT_MODE) packet.param7);
#endif
            break;

        case MAV_CMD_MISSION_START:
            plane.set_mode(AUTO);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_PREFLIGHT_CALIBRATION:
            plane.in_calibration = true;
            if (is_equal(packet.param1,1.0f)) {
                plane.ins.init_gyro();
                if (plane.ins.gyro_calibrated_ok_all()) {
                    plane.ahrs.reset_gyro_drift();
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
                }
            } else if (is_equal(packet.param3,1.0f)) {
                plane.init_barometer();
                if (plane.airspeed.enabled()) {
                    plane.zero_airspeed(false);
                }
                result = MAV_RESULT_ACCEPTED;
            } else if (is_equal(packet.param4,1.0f)) {
                plane.trim_radio();
                result = MAV_RESULT_ACCEPTED;
            } else if (is_equal(packet.param5,1.0f)) {
                float trim_roll, trim_pitch;
                AP_InertialSensor_UserInteract_MAVLink interact(this);
                // start with gyro calibration
                plane.ins.init_gyro();
                // reset ahrs gyro bias
                if (plane.ins.gyro_calibrated_ok_all()) {
                    plane.ahrs.reset_gyro_drift();
                }
                if(plane.ins.calibrate_accel(&interact, trim_roll, trim_pitch)) {
                    // reset ahrs's trim to suggested values from calibration routine
                    plane.ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
                }
            } else if (is_equal(packet.param5,2.0f)) {
                // start with gyro calibration
                plane.ins.init_gyro();
                // accel trim
                float trim_roll, trim_pitch;
                if(plane.ins.calibrate_trim(trim_roll, trim_pitch)) {
                    // reset ahrs's trim to suggested values from calibration routine
                    plane.ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
                }
            }
            else {
                    send_text(MAV_SEVERITY_WARNING, "Unsupported preflight calibration");
            }
            plane.in_calibration = false;
            break;

        case MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
            if (is_equal(packet.param1,2.0f)) {
                // save first compass's offsets
                plane.compass.set_and_save_offsets(0, packet.param2, packet.param3, packet.param4);
                result = MAV_RESULT_ACCEPTED;
            }
            if (is_equal(packet.param1,5.0f)) {
                // save secondary compass's offsets
                plane.compass.set_and_save_offsets(1, packet.param2, packet.param3, packet.param4);
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_COMPONENT_ARM_DISARM:
            if (is_equal(packet.param1,1.0f)) {
                // run pre_arm_checks and arm_checks and display failures
                if (plane.arm_motors(AP_Arming::MAVLINK)) {
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
                }
            } else if (is_zero(packet.param1))  {
                if (plane.disarm_motors()) {
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
                }
            } else {
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_GET_HOME_POSITION:
            if (plane.home_is_set != HOME_UNSET) {
                send_home(plane.ahrs.get_home());
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_SET_MODE:
            switch ((uint16_t)packet.param1) {
            case MAV_MODE_MANUAL_ARMED:
            case MAV_MODE_MANUAL_DISARMED:
                plane.set_mode(MANUAL);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_MODE_AUTO_ARMED:
            case MAV_MODE_AUTO_DISARMED:
                plane.set_mode(AUTO);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_MODE_STABILIZE_DISARMED:
            case MAV_MODE_STABILIZE_ARMED:
                plane.set_mode(FLY_BY_WIRE_A);
                result = MAV_RESULT_ACCEPTED;
                break;

            default:
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_DO_SET_SERVO:
            if (plane.ServoRelayEvents.do_set_servo(packet.param1, packet.param2)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_REPEAT_SERVO:
            if (plane.ServoRelayEvents.do_repeat_servo(packet.param1, packet.param2, packet.param3, packet.param4*1000)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_SET_RELAY:
            if (plane.ServoRelayEvents.do_set_relay(packet.param1, packet.param2)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_REPEAT_RELAY:
            if (plane.ServoRelayEvents.do_repeat_relay(packet.param1, packet.param2, packet.param3*1000)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            if (is_equal(packet.param1,1.0f) || is_equal(packet.param1,3.0f)) {
                // when packet.param1 == 3 we reboot to hold in bootloader
                hal.scheduler->reboot(is_equal(packet.param1,3.0f));
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_LAND_START:
            result = MAV_RESULT_FAILED;
            
            // attempt to switch to next DO_LAND_START command in the mission
            if (plane.jump_to_landing_sequence()) {
                result = MAV_RESULT_ACCEPTED;
            } 
            break;

        case MAV_CMD_DO_GO_AROUND:
            result = MAV_RESULT_FAILED;

            //Not allowing go around at FLIGHT_LAND_FINAL stage on purpose --
            //if plane is close to the ground a go around coudld be dangerous.
            if (plane.flight_stage == AP_SpdHgtControl::FLIGHT_LAND_APPROACH) {
                //Just tell the autopilot we're done landing so it will 
                //proceed to the next mission item.  If there is no next mission
                //item the plane will head to home point and loiter.
                plane.auto_state.commanded_go_around = true;
               
                result = MAV_RESULT_ACCEPTED;
                plane.gcs_send_text(MAV_SEVERITY_INFO,"Go around command accepted");
            } else {
                plane.gcs_send_text(MAV_SEVERITY_NOTICE,"Rejected go around command");
            }
            break;

        case MAV_CMD_DO_FENCE_ENABLE:
            result = MAV_RESULT_ACCEPTED;
            
            if (!plane.geofence_present()) {
                result = MAV_RESULT_FAILED;
            } switch((uint16_t)packet.param1) {
                case 0:
                    if (! plane.geofence_set_enabled(false, GCS_TOGGLED)) {
                        result = MAV_RESULT_FAILED;
                    }
                    break;
                case 1:
                    if (! plane.geofence_set_enabled(true, GCS_TOGGLED)) {
                        result = MAV_RESULT_FAILED; 
                    }
                    break;
                case 2: //disable fence floor only 
                    if (! plane.geofence_set_floor_enabled(false)) {
                        result = MAV_RESULT_FAILED;
                    } else {
                        plane.gcs_send_text(MAV_SEVERITY_NOTICE,"Fence floor disabled");
                    }
                    break;
                default:
                    result = MAV_RESULT_FAILED;
                    break;
            }
            break;

        case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES: {
            if (is_equal(packet.param1,1.0f)) {
                plane.gcs[chan-MAVLINK_COMM_0].send_autopilot_version(FIRMWARE_VERSION);
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_SET_HOME:
            // param1 : use current (1=use current location, 0=use specified location)
            // param5 : latitude
            // param6 : longitude
            // param7 : altitude (absolute)
            result = MAV_RESULT_FAILED; // assume failure
            if (is_equal(packet.param1,1.0f)) {
                plane.init_home();
            } else {
                if (is_zero(packet.param5) && is_zero(packet.param6) && is_zero(packet.param7)) {
                    // don't allow the 0,0 position
                    break;
                }
                // sanity check location
                if (fabsf(packet.param5) > 90.0f || fabsf(packet.param6) > 180.0f) {
                    break;
                }
                Location new_home_loc {};
                new_home_loc.lat = (int32_t)(packet.param5 * 1.0e7f);
                new_home_loc.lng = (int32_t)(packet.param6 * 1.0e7f);
                new_home_loc.alt = (int32_t)(packet.param7 * 100.0f);
                plane.ahrs.set_home(new_home_loc);
                plane.home_is_set = HOME_SET_NOT_LOCKED;
                plane.Log_Write_Home_And_Origin();
                GCS_MAVLINK::send_home_all(new_home_loc);
                result = MAV_RESULT_ACCEPTED;
                plane.gcs_send_text_fmt(MAV_SEVERITY_INFO, "Set HOME to %.6f %.6f at %um",
                                        (double)(new_home_loc.lat*1.0e-7f),
                                        (double)(new_home_loc.lng*1.0e-7f),
                                        (uint32_t)(new_home_loc.alt*0.01f));
            }
            break;
        }


#if PARACHUTE == ENABLED
        case MAV_CMD_DO_PARACHUTE:
            break;
#endif

        default:
            break;
        }

        mavlink_msg_command_ack_send_buf(
            msg,
            chan,
            packet.command,
            result);

        break;
    }


    case MAVLINK_MSG_ID_SET_MODE:
    {
        handle_set_mode(msg, FUNCTOR_BIND(&plane, &Plane::mavlink_set_mode, bool, uint8_t));
        break;
    }

    // GCS request the full list of commands, we return just the number and leave the GCS to then request each command individually
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
    {
        handle_mission_request_list(plane.mission, msg);
        break;
    }


    // XXX read a WP from EEPROM and send it to the GCS
    case MAVLINK_MSG_ID_MISSION_REQUEST:
    {
        handle_mission_request(plane.mission, msg);
        break;
    }


    case MAVLINK_MSG_ID_MISSION_ACK:
    {
        // nothing to do
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    {
        // mark the firmware version in the tlog
        send_text(MAV_SEVERITY_INFO, FIRMWARE_STRING);

#if defined(PX4_GIT_VERSION) && defined(NUTTX_GIT_VERSION)
        send_text(MAV_SEVERITY_INFO, "PX4: " PX4_GIT_VERSION " NuttX: " NUTTX_GIT_VERSION);
#endif
        handle_param_request_list(msg);
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    {
        handle_param_request_read(msg);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
    {
        handle_mission_clear_all(plane.mission, msg);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
    {
        // disable cross-track when user asks for WP change, to
        // prevent unexpected flight paths
        plane.auto_state.next_wp_no_crosstrack = true;
        handle_mission_set_current(plane.mission, msg);
        if (plane.control_mode == AUTO && plane.mission.state() == AP_Mission::MISSION_STOPPED) {
            plane.mission.resume();
        }
        break;
    }

    // GCS provides the full number of commands it wishes to upload
    //  individual commands will then be sent from the GCS using the MAVLINK_MSG_ID_MISSION_ITEM message
    case MAVLINK_MSG_ID_MISSION_COUNT:
    {
        handle_mission_count(plane.mission, msg);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
    {
        handle_mission_write_partial_list(plane.mission, msg);
        break;
    }

    // GCS has sent us a mission item, store to EEPROM
    case MAVLINK_MSG_ID_MISSION_ITEM:
    {
        if (handle_mission_item(msg, plane.mission)) {
            plane.DataFlash.Log_Write_EntireMission(plane.mission);
        }
        break;
    }

    case MAVLINK_MSG_ID_PARAM_SET:
    {
        handle_param_set(msg, &plane.DataFlash);
        break;
    }

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
    {
        // allow override of RC channel values for HIL
        // or for complete GCS control of switch position
        // and RC PWM values.
        if(msg->sysid != plane.g.sysid_my_gcs) break;                         // Only accept control from our gcs
        mavlink_rc_channels_override_t packet;
        int16_t v[8];
        mavlink_msg_rc_channels_override_decode(msg, &packet);

        v[0] = packet.chan1_raw;
        v[1] = packet.chan2_raw;
        v[2] = packet.chan3_raw;
        v[3] = packet.chan4_raw;
        v[4] = packet.chan5_raw;
        v[5] = packet.chan6_raw;
        v[6] = packet.chan7_raw;
        v[7] = packet.chan8_raw;

        if (hal.rcin->set_overrides(v, 8)) {
            plane.failsafe.last_valid_rc_ms = AP_HAL::millis();
        }

        // a RC override message is consiered to be a 'heartbeat' from
        // the ground station for failsafe purposes
        plane.failsafe.last_heartbeat_ms = AP_HAL::millis();
        break;
    }

    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        // We keep track of the last time we received a heartbeat from
        // our GCS for failsafe purposes
        if (msg->sysid != plane.g.sysid_my_gcs) break;
        plane.failsafe.last_heartbeat_ms = AP_HAL::millis();
        break;
    }

    case MAVLINK_MSG_ID_HIL_STATE:
    {
#if HIL_SUPPORT
        if (plane.g.hil_mode != 1) {
            break;
        }
        mavlink_hil_state_t packet;
        mavlink_msg_hil_state_decode(msg, &packet);

        last_hil_state = packet;

        // set gps hil sensor
        Location loc;
        memset(&loc, 0, sizeof(loc));
        loc.lat = packet.lat;
        loc.lng = packet.lon;
        loc.alt = packet.alt/10;
        Vector3f vel(packet.vx, packet.vy, packet.vz);
        vel *= 0.01f;

        // setup airspeed pressure based on 3D speed, no wind
        plane.airspeed.setHIL(sq(vel.length()) / 2.0f + 2013);

        plane.gps.setHIL(0, AP_GPS::GPS_OK_FIX_3D,
                         packet.time_usec/1000,
                         loc, vel, 10, 0, true);

        // rad/sec
        Vector3f gyros;
        gyros.x = packet.rollspeed;
        gyros.y = packet.pitchspeed;
        gyros.z = packet.yawspeed;

        // m/s/s
        Vector3f accels;
        accels.x = packet.xacc * GRAVITY_MSS*0.001f;
        accels.y = packet.yacc * GRAVITY_MSS*0.001f;
        accels.z = packet.zacc * GRAVITY_MSS*0.001f;

        plane.ins.set_gyro(0, gyros);
        plane.ins.set_accel(0, accels);

        plane.barometer.setHIL(packet.alt*0.001f);
        plane.compass.setHIL(0, packet.roll, packet.pitch, packet.yaw);
        plane.compass.setHIL(1, packet.roll, packet.pitch, packet.yaw);

        // cope with DCM getting badly off due to HIL lag
        if (plane.g.hil_err_limit > 0 &&
            (fabsf(packet.roll - plane.ahrs.roll) > ToRad(plane.g.hil_err_limit) ||
             fabsf(packet.pitch - plane.ahrs.pitch) > ToRad(plane.g.hil_err_limit) ||
             wrap_PI(fabsf(packet.yaw - plane.ahrs.yaw)) > ToRad(plane.g.hil_err_limit))) {
            plane.ahrs.reset_attitude(packet.roll, packet.pitch, packet.yaw);
        }
#endif
        break;
    }

    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
    case MAVLINK_MSG_ID_LOG_ERASE:
        plane.in_log_download = true;
        /* no break */
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
        if (!plane.in_mavlink_delay) {
            handle_log_message(msg, plane.DataFlash);
        }
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
        plane.in_log_download = false;
        if (!plane.in_mavlink_delay) {
            handle_log_message(msg, plane.DataFlash);
        }
        break;

    case MAVLINK_MSG_ID_SERIAL_CONTROL:
        handle_serial_control(msg, plane.gps);
        break;

    case MAVLINK_MSG_ID_GPS_INJECT_DATA:
        handle_gps_inject(msg, plane.gps);
        break;

    case MAVLINK_MSG_ID_TERRAIN_DATA:
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
#if AP_TERRAIN_AVAILABLE
        plane.terrain.handle_data(chan, msg);
#endif
        break;

    case MAVLINK_MSG_ID_SET_HOME_POSITION:
    {
        mavlink_set_home_position_t packet;
        mavlink_msg_set_home_position_decode(msg, &packet);
        if((packet.latitude == 0) && (packet.longitude == 0) && (packet.altitude == 0)) {
            // don't allow the 0,0 position
            break;
        }
        // sanity check location
        if (labs(packet.latitude) > 90*10e7 || labs(packet.longitude) > 180 * 10e7) {
            break;
        }
        Location new_home_loc {};
        new_home_loc.lat = packet.latitude;
        new_home_loc.lng = packet.longitude;
        new_home_loc.alt = packet.altitude * 100;
        plane.ahrs.set_home(new_home_loc);
        plane.home_is_set = HOME_SET_NOT_LOCKED;
        plane.Log_Write_Home_And_Origin();
        GCS_MAVLINK::send_home_all(new_home_loc);
        plane.gcs_send_text_fmt(MAV_SEVERITY_INFO, "Set HOME to %.6f %.6f at %um",
                                (double)(new_home_loc.lat*1.0e-7f),
                                (double)(new_home_loc.lng*1.0e-7f),
                                (uint32_t)(new_home_loc.alt*0.01f));
        break;
    }

    case MAVLINK_MSG_ID_ADSB_VEHICLE:
        plane.adsb.update_vehicle(msg);
        break;
    } // end switch
} // end handle mavlink

/*
 *  a delay() callback that processes MAVLink packets. We set this as the
 *  callback in long running library initialisation routines to allow
 *  MAVLink to process packets while waiting for the initialisation to
 *  complete
 */
void Plane::mavlink_delay_cb()
{
    static uint32_t last_1hz, last_50hz, last_5s;
    if (!gcs[0].initialised || in_mavlink_delay) return;

    in_mavlink_delay = true;

    uint32_t tnow = millis();
    if (tnow - last_1hz > 1000) {
        last_1hz = tnow;
        gcs_send_message(MSG_HEARTBEAT);
        gcs_send_message(MSG_EXTENDED_STATUS1);
    }
    if (tnow - last_50hz > 20) {
        last_50hz = tnow;
        gcs_update();
        gcs_data_stream_send();
        notify.update();
    }
    if (tnow - last_5s > 5000) {
        last_5s = tnow;
        gcs_send_text(MAV_SEVERITY_INFO, "Initialising APM");
    }
    check_usb_mux();

    in_mavlink_delay = false;
}

/*
 *  send a message on both GCS links
 */
void Plane::gcs_send_message(enum ap_message id)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].send_message(id);
        }
    }
}

/*
 *  send a mission item reached message and load the index before the send attempt in case it may get delayed
 */
void Plane::gcs_send_mission_item_reached_message(uint16_t mission_index)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].mission_item_reached_index = mission_index;
            gcs[i].send_message(MSG_MISSION_ITEM_REACHED);
        }
    }
}

/*
 *  send data streams in the given rate range on both links
 */
void Plane::gcs_data_stream_send(void)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].data_stream_send();
        }
    }
}

/*
 *  look for incoming commands on the GCS links
 */
void Plane::gcs_update(void)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
#if CLI_ENABLED == ENABLED
            gcs[i].update(g.cli_enabled == 1 ? FUNCTOR_BIND_MEMBER(&Plane::run_cli, void, AP_HAL::UARTDriver *):NULL);
#else
            gcs[i].update(NULL);
#endif
        }
    }
}

void Plane::gcs_send_text(MAV_SEVERITY severity, const char *str)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].send_text(severity, str);
        }
    }
#if LOGGING_ENABLED == ENABLED
    DataFlash.Log_Write_Message(str);
#endif
}

/*
 *  send a low priority formatted message to the GCS
 *  only one fits in the queue, so if you send more than one before the
 *  last one gets into the serial buffer then the old one will be lost
 */
void Plane::gcs_send_text_fmt(MAV_SEVERITY severity, const char *fmt, ...)
{
    va_list arg_list;
    gcs[0].pending_status.severity = (uint8_t)severity;
    va_start(arg_list, fmt);
    hal.util->vsnprintf((char *)gcs[0].pending_status.text,
            sizeof(gcs[0].pending_status.text), fmt, arg_list);
    va_end(arg_list);
#if LOGGING_ENABLED == ENABLED
    DataFlash.Log_Write_Message(gcs[0].pending_status.text);
#endif
    gcs[0].send_message(MSG_STATUSTEXT);
    for (uint8_t i=1; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].pending_status = gcs[0].pending_status;
            gcs[i].send_message(MSG_STATUSTEXT);
        }
    }
}

/*
  send airspeed calibration data
 */
void Plane::gcs_send_airspeed_calibration(const Vector3f &vg)
{
}

/**
   retry any deferred messages
 */
void Plane::gcs_retry_deferred(void)
{
    gcs_send_message(MSG_RETRY_DEFERRED);
}
