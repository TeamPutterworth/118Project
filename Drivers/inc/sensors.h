/*
 * File:   sensors.h
 * Author: jdgrant
 *
 * This file will contain helper functions for driving the sensors and interfacing with them.
 * There is also an init function for initializing the various sensors.
 * 
 */

#ifndef _SENSORS_H   
#define _SENSORS_H

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



void sensorsInit(); 

uint8_t readTrackWire();

uint8_t muxSelTrackWire();

uint16_t * readTapeSensors();

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

#endif /* _SENSORS_H */
