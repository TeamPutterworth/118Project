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



uint8_t readTrackWire()
{
    uint8_t retVal = (IO_PortsReadPort(PORTW) & TRACKWIRE_OUTPUT) >> 7;
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
    retVal = retVal | (IO_PortsReadPort(PORTW) & B_BUMPER);
    
    return (retVal >> 3);
}

void sensorsInit()
{
    //Init for track wire and beacon detector
    IO_PortsSetPortInputs(PORTW,TRACKWIRE_OUTPUT | BEACONDETECT_OUTPUT );

    // Init for Bumpers
    IO_PortsSetPortInputs(PORTW,FR_BUMPER|FL_BUMPER|B_BUMPER);
    
    // Init for multiplexer select
    IO_PortsSetPortOutputs(PORTZ,FR_BUMPER|FL_BUMPER|B_BUMPER);
    
}

uint8_t readBeaconDetector()

    {
    uint8_t retVal = (IO_PortsReadPort(PORTW) & BEACONDETECT_OUTPUT ) >> 8;
//#ifdef DEBUG
   // printf("\r\nPort Y val is: %d,Track Wire: %d", IO_PortsReadPort(PORTY),retVal);
//#endif
    return retVal;
}
    
   
