/*
 * File:   sensors.h
 * Author: jdgrant
 *
 * This file will contain helper functions for driving the sensors and interfacing with them.
 * There is also an init function for initializing the various sensors.
 * 
 */

#include "IO_Ports.h"

#ifndef _SENSORS_H   
#define _SENSORS_H

// Tape sensors
#define TS_FR 0x01
#define TS_FL 0x02
#define TS_FM 0x04
#define TS_BR 0x08
#define TS_BL 0x10
#define FR_SH 0
#define FL_SH 1
#define FM_SH 2
#define BR_SH 3
#define BL_SH 4
// Trackwire and beacon
#define TRACKWIRE_OUTPUT PIN7
#define BEACONDETECT_OUTPUT PIN8 
#define SENSOR_PORT PORTW
#define TW_F 0x01
#define TW_B 0x02
// Bumpers
#define FR_BUMPER PIN3
#define FL_BUMPER PIN4
#define B_BUMPER PIN5
#define BUMPER_PORT PORTW

// Mux
#define MUX_SELECT_A PIN11
#define MUX_SELECT_A_SH 11
#define MUX_SELECT_B PIN12
#define MUX_SELECT_B_SH 12
#define MUX_PORT PORTZ
// Servos
#define RC_SERVO_UNLOADING RC_PORTY06
#define RC_SERVO_BRIDGE RC_PORTY07

void sensorsInit(); 

uint8_t readTrackWire();

/*
 * desc: input 8-bit value with 
 *      sel 0 = bit 0
 *      sel 1 = bit 1
 */
void muxSelTrackWire(uint8_t selectMask);

uint8_t readBeaconDetector();

/*
 * desc: return 8-bit value with values of each bumper bit masked in.
 * 
 * ret: 8-bit value with bumper values masked
 * 8'bxxxx LRLR
 * where the first two LSB are the front bumpers and 
 * next are the back bumpers
 */
uint8_t readBumpSensors();

void setPulseUnloadingServo(uint16_t pulse);

void setPulseBridgeServo(uint16_t pulse);

#endif /* _SENSORS_H */
