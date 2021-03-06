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
#include "RC_Servo.h"
#include "sensors.h"
#include <stdio.h>

//#define DEBUG
uint8_t readTrackWire()
{
    uint8_t retVal = (IO_PortsReadPort(SENSOR_PORT) & TRACKWIRE_OUTPUT) >> 7;
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
    
    retVal = retVal | (IO_PortsReadPort(BUMPER_PORT) & FR_BUMPER);
    retVal = retVal | (IO_PortsReadPort(BUMPER_PORT) & FL_BUMPER);
    retVal = retVal | (IO_PortsReadPort(BUMPER_PORT) & B_BUMPER);
    
    return (retVal >> 3);
}

void sensorsInit()
{
    //Init for track wire and beacon detector
    IO_PortsSetPortInputs(SENSOR_PORT,TRACKWIRE_OUTPUT | BEACONDETECT_OUTPUT );

    // Init for Bumpers
    IO_PortsSetPortInputs(BUMPER_PORT,FR_BUMPER|FL_BUMPER|B_BUMPER);
    
    // Init for multiplexer select
    IO_PortsSetPortOutputs(MUX_PORT,MUX_SELECT_A);
    
    // Init for servos
    RC_AddPins(RC_SERVO_UNLOADING|RC_SERVO_BRIDGE);
    
}

uint8_t readBeaconDetector()

    {
    uint8_t retVal = (IO_PortsReadPort(SENSOR_PORT) & BEACONDETECT_OUTPUT ) >> 8;
//#ifdef DEBUG
   // printf("\r\nPort Y val is: %d,Track Wire: %d", IO_PortsReadPort(PORTY),retVal);
//#endif
    return retVal;
}

/*
 * desc: input 8-bit value with 
 *      sel 0 = bit 0
 *      sel 1 = bit 1
 */
void muxSelTrackWire(uint8_t selectMask)
{
    if (selectMask)
    {
        IO_PortsWritePort(MUX_PORT,IO_PortsReadPort(MUX_PORT) | MUX_SELECT_A); // This sets select A on the mux
    }
    else
    {
        IO_PortsWritePort(MUX_PORT,IO_PortsReadPort(MUX_PORT) & ~MUX_SELECT_A); // This clears select A on the mux
    }
    return;
}

void setPulseUnloadingServo(uint16_t pulse)
{  
    RC_SetPulseTime(RC_SERVO_UNLOADING,pulse);
    return;
}

void setPulseBridgeServo(uint16_t pulse)
{
    RC_SetPulseTime(RC_SERVO_BRIDGE,pulse);
    return;
}
   
