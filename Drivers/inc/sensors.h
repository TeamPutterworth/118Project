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
