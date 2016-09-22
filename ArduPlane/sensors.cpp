// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"
#include <AP_RSSI/AP_RSSI.h>

void Plane::init_barometer(void)
{
    gcs_send_text(MAV_SEVERITY_INFO, "Calibrating barometer");
    barometer.calibrate();

    gcs_send_text(MAV_SEVERITY_INFO, "Barometer calibration complete");
}

void Plane::init_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
    rangefinder.init();
#endif
}

/*
  read the rangefinder and update height estimate
 */
void Plane::read_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED

    // notify the rangefinder of our approximate altitude above ground to allow it to power on
    // during low-altitude flight when configured to power down during higher-altitude flight
    float height;
#if AP_TERRAIN_AVAILABLE
    if (terrain.status() == AP_Terrain::TerrainStatusOK && terrain.height_above_terrain(height, true)) {
        rangefinder.set_estimated_terrain_height(height);
    } else
#endif
    {
        // use the best available alt estimate via baro above home
        if (flight_stage == AP_SpdHgtControl::FLIGHT_LAND_APPROACH ||
            flight_stage == AP_SpdHgtControl::FLIGHT_LAND_FINAL) {
            // ensure the rangefinder is powered-on when land alt is higher than home altitude.
            // This is done using the target alt which we know is below us and we are sinking to it
            height = height_above_target();
        } else {
            // otherwise just use the best available baro estimate above home.
            height = relative_altitude();
        }
        rangefinder.set_estimated_terrain_height(height);
    }

    rangefinder.update();

    if (should_log(MASK_LOG_SONAR))
        Log_Write_Sonar();

    rangefinder_height_update();
#endif
}

/*
  calibrate compass
*/
void Plane::compass_cal_update() {
    if (!hal.util->get_soft_armed()) {
        compass.compass_cal_update();
    }
}

/*
  ask airspeed sensor for a new value
 */
void Plane::read_airspeed(void)
{
    if (airspeed.enabled()) {
        airspeed.read();
        if (should_log(MASK_LOG_IMU)) {
            Log_Write_Airspeed();
        }
        calc_airspeed_errors();

        // supply a new temperature to the barometer from the digital
        // airspeed sensor if we can
        float temperature;
        if (airspeed.get_temperature(temperature)) {
            barometer.set_external_temperature(temperature);
        }
    }

    // update smoothed airspeed estimate
    float aspeed;
    if (ahrs.airspeed_estimate(&aspeed)) {
//        smoothed_airspeed = smoothed_airspeed * 0.8f + aspeed * 0.2f;
        smoothed_airspeed = airspeed_lpf.filter(aspeed);
    }
}

void Plane::zero_airspeed(bool in_startup)
{
    airspeed.calibrate(in_startup);
    read_airspeed();
    // update barometric calibration with new airspeed supplied temperature
    barometer.update_calibration();
    gcs_send_text(MAV_SEVERITY_INFO,"Zero airspeed calibrated");
}

// read_battery - reads battery voltage and current and invokes failsafe
// should be called at 10hz
void Plane::read_battery(void)
{
    battery.read();
    compass.set_current(battery.current_amps());

    if (!usb_connected && 
        hal.util->get_soft_armed() &&
        battery.exhausted(g.fs_batt_voltage, g.fs_batt_mah)) {
        low_battery_event();
    }
}

// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void Plane::read_receiver_rssi(void)
{
    receiver_rssi = rssi.read_receiver_rssi_uint8();
}

/*
  update RPM sensors
 */
void Plane::rpm_update(void)
{
    rpm_sensor.update();
    if (rpm_sensor.healthy(0) || rpm_sensor.healthy(1)) {
        if (should_log(MASK_LOG_RC)) {
            DataFlash.Log_Write_RPM(rpm_sensor);
        }
    }
}

// True airspeed calculation.
float Plane::get_true_airspeed()
{
	/***************
	// Uses TAS formula from http://www.mathpages.com/home/kmath282/kmath282.htm
	const float C = 0.286; // (g-1)/g where g is ratio of specific heats for air.
	const float R2 = 16.629; // Twice the gas constant.
	float T, pressure_ratio;

	// pressure ratio (stagnation/static)
	pressure_ratio = airspeed.get_differential_pressure()/barometer.get_pressure() + 1.f;

	// temperature in Kelvin
    if( ! airspeed.get_temperature(T)){
    	T = barometer.get_temperature();
    }
	T += 273.15f;

	return sqrt(R2*T/C*(pow(pressure_ratio,C)-1.0f));
	**************/

	return airspeed.get_EAS2TAS() * smoothed_airspeed;

}


// Calculate total energy compensated vertical speed.
void Plane::compensated_vario()
{
	static uint32_t t_then=0;
	uint32_t t_now;
	float das_dt; // time derivative of airspeed

	/* Damping factor: tuning parameter used to reduce the effect of energy
	   compensation on the vario signal. */
	const float te_damping_factor = 0.5;

	// Get vertical acceleration component from the NED
	Vector3f v_ned;
	// Vertical velocity component
	if( ! ahrs.get_velocity_NED(v_ned)){
    	v_ned.zero();
    }

    // check for new airspeed update
    t_now = airspeed.last_update_ms();
    if( t_now > t_then){
		// airspeed differential
		airspeed_tas = get_true_airspeed();
	    airspeed_derivative_lpf.update(airspeed_tas, t_now*1000);
		t_then = t_now;
    }

	das_dt = airspeed_derivative_lpf.slope() * 1.0e6f;
	vario_TE = -v_ned.z +
			(te_damping_factor * airspeed_tas * das_dt / GRAVITY_MSS);
}

void Plane::update_audio_vario()
{
	int16_t vario_input, vspeed_limit;

	if(xcsoar_data.flying && !xcsoar_data.circling){
		// speed-to-fly director mode
		float delta_s = smoothed_airspeed - xcsoar_data.speed_to_fly;
		if(delta_s >= 0.0){
			vspeed_limit = audio_vario.get_vario_limit_upper();
		}else{
			vspeed_limit = -audio_vario.get_vario_limit_lower();
		}
		vario_input = (int16_t) ( (delta_s/7.7) * (float)vspeed_limit);
//		debug_dummy1 = smoothed_airspeed;
//		debug_dummy2 = xcsoar_data.speed_to_fly;
//		debug_dummy3 = vario_input;
	}else{
		// ordinary vario mode
		vario_input = (int16_t)(vario_TE * 100);
//		debug_dummy1 = debug_dummy2 = debug_dummy3 = 0.0;
	}
	audio_vario.update(vario_input);
}
