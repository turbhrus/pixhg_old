/** @file
 *	@brief MAVLink comm protocol testsuite generated from PIXHAWK.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef PIXHAWK_TESTSUITE_H
#define PIXHAWK_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_PIXHAWK(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_PIXHAWK(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_data16(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_data16_t packet_in = {
		5,72,{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154 }
    };
	mavlink_data16_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.type = packet_in.type;
        	packet1.len = packet_in.len;
        
        	mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*16);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_data16_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_data16_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_data16_pack(system_id, component_id, &msg , packet1.type , packet1.len , packet1.data );
	mavlink_msg_data16_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_data16_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.type , packet1.len , packet1.data );
	mavlink_msg_data16_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_data16_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_data16_send(MAVLINK_COMM_1 , packet1.type , packet1.len , packet1.data );
	mavlink_msg_data16_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_data32(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_data32_t packet_in = {
		5,72,{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170 }
    };
	mavlink_data32_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.type = packet_in.type;
        	packet1.len = packet_in.len;
        
        	mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*32);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_data32_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_data32_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_data32_pack(system_id, component_id, &msg , packet1.type , packet1.len , packet1.data );
	mavlink_msg_data32_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_data32_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.type , packet1.len , packet1.data );
	mavlink_msg_data32_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_data32_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_data32_send(MAVLINK_COMM_1 , packet1.type , packet1.len , packet1.data );
	mavlink_msg_data32_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_data64(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_data64_t packet_in = {
		5,72,{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202 }
    };
	mavlink_data64_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.type = packet_in.type;
        	packet1.len = packet_in.len;
        
        	mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*64);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_data64_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_data64_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_data64_pack(system_id, component_id, &msg , packet1.type , packet1.len , packet1.data );
	mavlink_msg_data64_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_data64_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.type , packet1.len , packet1.data );
	mavlink_msg_data64_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_data64_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_data64_send(MAVLINK_COMM_1 , packet1.type , packet1.len , packet1.data );
	mavlink_msg_data64_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_data96(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_data96_t packet_in = {
		5,72,{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234 }
    };
	mavlink_data96_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.type = packet_in.type;
        	packet1.len = packet_in.len;
        
        	mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*96);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_data96_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_data96_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_data96_pack(system_id, component_id, &msg , packet1.type , packet1.len , packet1.data );
	mavlink_msg_data96_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_data96_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.type , packet1.len , packet1.data );
	mavlink_msg_data96_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_data96_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_data96_send(MAVLINK_COMM_1 , packet1.type , packet1.len , packet1.data );
	mavlink_msg_data96_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_pixhawk_hg_fast(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_pixhawk_hg_fast_t packet_in = {
		963497464,45.0,73.0,101.0,129.0,157.0,18483
    };
	mavlink_pixhawk_hg_fast_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.gps_time = packet_in.gps_time;
        	packet1.xacc = packet_in.xacc;
        	packet1.yacc = packet_in.yacc;
        	packet1.zacc = packet_in.zacc;
        	packet1.roll = packet_in.roll;
        	packet1.pitch = packet_in.pitch;
        	packet1.hdg = packet_in.hdg;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pixhawk_hg_fast_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_pixhawk_hg_fast_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pixhawk_hg_fast_pack(system_id, component_id, &msg , packet1.gps_time , packet1.xacc , packet1.yacc , packet1.zacc , packet1.roll , packet1.pitch , packet1.hdg );
	mavlink_msg_pixhawk_hg_fast_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pixhawk_hg_fast_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.gps_time , packet1.xacc , packet1.yacc , packet1.zacc , packet1.roll , packet1.pitch , packet1.hdg );
	mavlink_msg_pixhawk_hg_fast_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_pixhawk_hg_fast_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pixhawk_hg_fast_send(MAVLINK_COMM_1 , packet1.gps_time , packet1.xacc , packet1.yacc , packet1.zacc , packet1.roll , packet1.pitch , packet1.hdg );
	mavlink_msg_pixhawk_hg_fast_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_pixhawk_hg_med(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_pixhawk_hg_med_t packet_in = {
		963497464,963497672,963497880,101.0,129.0,18275,18379,18483,18587
    };
	mavlink_pixhawk_hg_med_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.lat = packet_in.lat;
        	packet1.lon = packet_in.lon;
        	packet1.alt = packet_in.alt;
        	packet1.indicated_airspeed = packet_in.indicated_airspeed;
        	packet1.true_airspeed = packet_in.true_airspeed;
        	packet1.vx = packet_in.vx;
        	packet1.vy = packet_in.vy;
        	packet1.vz = packet_in.vz;
        	packet1.energy_vario = packet_in.energy_vario;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pixhawk_hg_med_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_pixhawk_hg_med_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pixhawk_hg_med_pack(system_id, component_id, &msg , packet1.lat , packet1.lon , packet1.alt , packet1.vx , packet1.vy , packet1.vz , packet1.energy_vario , packet1.indicated_airspeed , packet1.true_airspeed );
	mavlink_msg_pixhawk_hg_med_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pixhawk_hg_med_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.lat , packet1.lon , packet1.alt , packet1.vx , packet1.vy , packet1.vz , packet1.energy_vario , packet1.indicated_airspeed , packet1.true_airspeed );
	mavlink_msg_pixhawk_hg_med_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_pixhawk_hg_med_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pixhawk_hg_med_send(MAVLINK_COMM_1 , packet1.lat , packet1.lon , packet1.alt , packet1.vx , packet1.vy , packet1.vz , packet1.energy_vario , packet1.indicated_airspeed , packet1.true_airspeed );
	mavlink_msg_pixhawk_hg_med_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_pixhawk_hg_slow(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_pixhawk_hg_slow_t packet_in = {
		93372036854775807ULL,963497880,101.0,129.0,157.0,185.0,18691,223,34,101
    };
	mavlink_pixhawk_hg_slow_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.gps_time_us = packet_in.gps_time_us;
        	packet1.gps_alt = packet_in.gps_alt;
        	packet1.temperature = packet_in.temperature;
        	packet1.wind_n = packet_in.wind_n;
        	packet1.wind_e = packet_in.wind_e;
        	packet1.wind_d = packet_in.wind_d;
        	packet1.eph = packet_in.eph;
        	packet1.gps_fix_type = packet_in.gps_fix_type;
        	packet1.satellites_visible = packet_in.satellites_visible;
        	packet1.humidity = packet_in.humidity;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pixhawk_hg_slow_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_pixhawk_hg_slow_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pixhawk_hg_slow_pack(system_id, component_id, &msg , packet1.gps_time_us , packet1.eph , packet1.gps_alt , packet1.gps_fix_type , packet1.satellites_visible , packet1.temperature , packet1.humidity , packet1.wind_n , packet1.wind_e , packet1.wind_d );
	mavlink_msg_pixhawk_hg_slow_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pixhawk_hg_slow_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.gps_time_us , packet1.eph , packet1.gps_alt , packet1.gps_fix_type , packet1.satellites_visible , packet1.temperature , packet1.humidity , packet1.wind_n , packet1.wind_e , packet1.wind_d );
	mavlink_msg_pixhawk_hg_slow_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_pixhawk_hg_slow_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pixhawk_hg_slow_send(MAVLINK_COMM_1 , packet1.gps_time_us , packet1.eph , packet1.gps_alt , packet1.gps_fix_type , packet1.satellites_visible , packet1.temperature , packet1.humidity , packet1.wind_n , packet1.wind_e , packet1.wind_d );
	mavlink_msg_pixhawk_hg_slow_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_xcsoar_calculated(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_xcsoar_calculated_t packet_in = {
		17.0,17443,151,218,29,96
    };
	mavlink_xcsoar_calculated_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.speed_to_fly = packet_in.speed_to_fly;
        	packet1.current_turnpoint = packet_in.current_turnpoint;
        	packet1.flying = packet_in.flying;
        	packet1.circling = packet_in.circling;
        	packet1.airspace_warning_idx = packet_in.airspace_warning_idx;
        	packet1.terrain_warning = packet_in.terrain_warning;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_xcsoar_calculated_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_xcsoar_calculated_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_xcsoar_calculated_pack(system_id, component_id, &msg , packet1.speed_to_fly , packet1.flying , packet1.circling , packet1.current_turnpoint , packet1.airspace_warning_idx , packet1.terrain_warning );
	mavlink_msg_xcsoar_calculated_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_xcsoar_calculated_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.speed_to_fly , packet1.flying , packet1.circling , packet1.current_turnpoint , packet1.airspace_warning_idx , packet1.terrain_warning );
	mavlink_msg_xcsoar_calculated_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_xcsoar_calculated_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_xcsoar_calculated_send(MAVLINK_COMM_1 , packet1.speed_to_fly , packet1.flying , packet1.circling , packet1.current_turnpoint , packet1.airspace_warning_idx , packet1.terrain_warning );
	mavlink_msg_xcsoar_calculated_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_PIXHAWK(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_data16(system_id, component_id, last_msg);
	mavlink_test_data32(system_id, component_id, last_msg);
	mavlink_test_data64(system_id, component_id, last_msg);
	mavlink_test_data96(system_id, component_id, last_msg);
	mavlink_test_pixhawk_hg_fast(system_id, component_id, last_msg);
	mavlink_test_pixhawk_hg_med(system_id, component_id, last_msg);
	mavlink_test_pixhawk_hg_slow(system_id, component_id, last_msg);
	mavlink_test_xcsoar_calculated(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // PIXHAWK_TESTSUITE_H
