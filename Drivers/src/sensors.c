/*
 * File:   sensors.c
 * Author: jdgrant
 *
 * This file will contain helper functions for driving the sensors and interfacing with them.
 * There is also an init function for initializing the various sensors.
 * 
 */

#include "BOARD.h"
#include "IO_Ports.h"
#include "AD.h"
#include "sensors.h"
#include <stdio.h>

//#define DEBUG
#define TRACKWIRE_OUTPUT PIN6
#define FR_BUMPER PIN3
#define FL_BUMPER PIN4
#define BR_BUMPER PIN5
#define BL_BUMPER PIN6
#define BEACONDETECT_OUTPUT PIN7 


uint8_t readTrackWire()
{
    uint8_t retVal = (IO_PortsReadPort(PORTY) & TRACKWIRE_OUTPUT) >> 6;
#ifdef DEBUG
    printf("\r\nPort Y val is: %d,Track Wire: %d", IO_PortsReadPort(PORTY),retVal);
#endif
    return retVal;
}

/*
 * desc: This function will read the bumper states into a 8-bit value
 * FR Bumper - Bit 0 
 * FL Bumper - Bit 1 
 * BR Bumper - Bit 2 
 * BL Bumper - Bit 3
 */
uint8_t readBumpers()
{
    uint8_t retVal = 0;
    
    retVal = retVal | (IO_PortsReadPort(PORTW) & FR_BUMPER);
    retVal = retVal | (IO_PortsReadPort(PORTW) & FL_BUMPER);
    retVal = retVal | (IO_PortsReadPort(PORTW) & BR_BUMPER);
    retVal = retVal | (IO_PortsReadPort(PORTW) & BL_BUMPER);
    
    return (retVal >> 3);
}

void sensorsInit()
{
    //Init for track wire and beacon detector
    IO_PortsSetPortInputs(PORTY,TRACKWIRE_OUTPUT | BEACONDETECT_OUTPUT );

    // Init for Bumpers
    IO_PortsSetPortInputs(PORTW,FR_BUMPER|FL_BUMPER|BR_BUMPER|BL_BUMPER);
}

uint8_t readBeaconDetector()

    {
    uint8_t retVal = (IO_PortsReadPort(PORTY) & BEACONDETECT_OUTPUT ) >> 7;
//#ifdef DEBUG
   // printf("\r\nPort Y val is: %d,Track Wire: %d", IO_PortsReadPort(PORTY),retVal);
//#endif
    return retVal;
}
    
   
