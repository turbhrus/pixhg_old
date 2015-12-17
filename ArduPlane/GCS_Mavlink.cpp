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

	case MSG_ATTITUDE:
	case MSG_LOCATION:
	case MSG_EXTENDED_STATUS1:
	case MSG_EXTENDED_STATUS2:
	case MSG_NAV_CONTROLLER_OUTPUT:
	case MSG_CURRENT_WAYPOINT:
	case MSG_VFR_HUD:
	case MSG_RADIO_OUT:
	case MSG_RADIO_IN:
	case MSG_RAW_IMU1:
	case MSG_RAW_IMU2:
	case MSG_RAW_IMU3:
	case MSG_GPS_RAW:
	case MSG_SYSTEM_TIME:
	case MSG_SERVO_OUT:
	case MSG_NEXT_WAYPOINT:
	case MSG_NEXT_PARAM:
	case MSG_STATUSTEXT:
	case MSG_LIMITS_STATUS:
	case MSG_FENCE_STATUS:
	case MSG_AHRS:
	case MSG_SIMSTATE:
	case MSG_HWSTATUS:
	case MSG_WIND:
	case MSG_RANGEFINDER:
	case MSG_TERRAIN:
	case MSG_BATTERY2:
	case MSG_CAMERA_FEEDBACK:
	case MSG_MOUNT_STATUS:
	case MSG_OPTICAL_FLOW:
	case MSG_GIMBAL_REPORT:
	case MSG_MAG_CAL_PROGRESS:
	case MSG_MAG_CAL_REPORT:
	case MSG_EKF_STATUS_REPORT:
	case MSG_LOCAL_POSITION:
	case MSG_PID_TUNING:
	case MSG_VIBRATION:
	case MSG_RPM:
	case MSG_MISSION_ITEM_REACHED:
	case MSG_RETRY_DEFERRED:
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
        case MAV_CMD_NAV_LOITER_UNLIM:
        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        case MAV_CMD_DO_SET_ROI:
        case MAV_CMD_DO_DIGICAM_CONFIGURE:
        case MAV_CMD_DO_DIGICAM_CONTROL:
        case MAV_CMD_DO_MOUNT_CONTROL:
        case MAV_CMD_MISSION_START:
        case MAV_CMD_DO_SET_SERVO:
        case MAV_CMD_DO_REPEAT_SERVO:
        case MAV_CMD_DO_SET_RELAY:
        case MAV_CMD_DO_REPEAT_RELAY:
        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
        case MAV_CMD_DO_LAND_START:
        case MAV_CMD_DO_GO_AROUND:
        case MAV_CMD_DO_FENCE_ENABLE:
        case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES: {
        case MAV_CMD_DO_SET_HOME:
        case MAV_CMD_COMPONENT_ARM_DISARM:
        case MAV_CMD_GET_HOME_POSITION:
        case MAV_CMD_DO_SET_MODE:
        case MAV_CMD_DO_PARACHUTE:
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
        }
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
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
    case MAVLINK_MSG_ID_MISSION_REQUEST:
    case MAVLINK_MSG_ID_MISSION_ACK:
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
    case MAVLINK_MSG_ID_MISSION_COUNT:
    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
    case MAVLINK_MSG_ID_MISSION_ITEM:
    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
    case MAVLINK_MSG_ID_HIL_STATE:
    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
    case MAVLINK_MSG_ID_LOG_ERASE:
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
    case MAVLINK_MSG_ID_SERIAL_CONTROL:
    case MAVLINK_MSG_ID_GPS_INJECT_DATA:
    case MAVLINK_MSG_ID_TERRAIN_DATA:
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
    case MAVLINK_MSG_ID_SET_HOME_POSITION:
    	break;
    case MAVLINK_MSG_ID_PARAM_SET:
    {
        handle_param_set(msg, &plane.DataFlash);
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

/**
   retry any deferred messages
 */
void Plane::gcs_retry_deferred(void)
{
    gcs_send_message(MSG_RETRY_DEFERRED);
}

void Plane::send_attitude(mavlink_channel_t chan){}
void Plane::send_fence_status(mavlink_channel_t chan){}
void Plane::send_extended_status1(mavlink_channel_t chan){}
void Plane::send_location(mavlink_channel_t chan){}
void Plane::send_nav_controller_output(mavlink_channel_t chan){}
void Plane::send_servo_out(mavlink_channel_t chan){}
void Plane::send_radio_out(mavlink_channel_t chan){}
void Plane::send_vfr_hud(mavlink_channel_t chan){}
void Plane::send_simstate(mavlink_channel_t chan){}
void Plane::send_hwstatus(mavlink_channel_t chan){}
void Plane::send_wind(mavlink_channel_t chan){}
void NOINLINE Plane::send_rpm(mavlink_channel_t chan){}
void Plane::send_pid_tuning(mavlink_channel_t chan){}
void Plane::send_rangefinder(mavlink_channel_t chan){}
void Plane::send_current_waypoint(mavlink_channel_t chan){}
void Plane::gcs_send_airspeed_calibration(const Vector3f &vg){}

