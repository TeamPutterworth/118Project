/*
 * File:   motor.c
 * Author: jdgrant
 *
 * Motor file for helper functions which will be used to perform basic movement
 * and configure motor speed.
 * 
 */

#include "BOARD.h"
#include "IO_Ports.h"
#include "LED.h"
#include "pwm.h"
#include "AD.h"
#include "motor.h"
#include <stdio.h>

//#define DEBUG
#define PWM_LEFT_MOTOR  PWM_PORTY10
#define PWM_RIGHT_MOTOR PWM_PORTY12
#define DIR_LEFT_MOTOR  PIN9 
#define DIR_RIGHT_MOTOR PIN11
#define RIGHT 1
#define LEFT 0

void motorInit()
{
    IO_PortsSetPortOutputs(PORTY, DIR_LEFT_MOTOR|DIR_RIGHT_MOTOR);
    PWM_AddPins(PWM_LEFT_MOTOR|PWM_RIGHT_MOTOR);
}

void pivotTurnRight()
{
	motorForward(LEFT);
	motorStop(RIGHT);
}

void pivotTurnLeft()
{
	motorForward(RIGHT);
	motorStop(LEFT);
}

void tankTurnRight()
{
	motorForward(RIGHT);
	motorForward(LEFT);
}
void tankTurnLeft()
{
	motorBackward(LEFT);
	motorBackward(RIGHT);
}

void moveBackward()
{
	motorForward(LEFT);
	motorBackward(RIGHT);
}

void moveForward()
{
	motorForward(RIGHT);
	motorBackward(LEFT);
}

void stopMoving()
{
	setMoveSpeed(0);
}

/*
* lr = 1 makes right wheel go forward
* lr = 0 makes left wheel go forward
*/
void motorForward(int lr)
{
	// sets IN0 = 0, IN1 = 1
	if (lr == RIGHT) 
	{
        IO_PortsWritePort(PORTY,IO_PortsReadPort(PORTY) | DIR_RIGHT_MOTOR); // Set Dir right motor = high (forward)
	}
	else
	{   
        IO_PortsWritePort(PORTY,IO_PortsReadPort(PORTY) | DIR_LEFT_MOTOR); // Set Dir left motor = high (forward)
	}
	return;
}

/*
* lr = 1 makes right wheel go backward
* lr = 0 makes left wheel go backward
*/
void motorBackward(int lr)
{
	// sets IN0 = 0, IN1 = 1
	if (lr == RIGHT) 
	{
		IO_PortsWritePort(PORTY,IO_PortsReadPort(PORTY) & ~DIR_RIGHT_MOTOR); // Set Dir right motor = low (backward)
	}
	else
	{
		IO_PortsWritePort(PORTY,IO_PortsReadPort(PORTY) & ~DIR_LEFT_MOTOR); // Set Dir left motor = low (backward)
	}
	return;
}

void motorStop(int lr)
{
	if (lr == RIGHT) 
	{
		PWM_SetDutyCycle(PWM_RIGHT_MOTOR,0);
	}
	else
	{
		PWM_SetDutyCycle(PWM_LEFT_MOTOR,0);
	}
	return;
}

/*
 * Right now speed should be duty cycle 0-100
 * This should be updated to take input of speed in cm/s
 * and calculate pwm required to supply enough voltage,
 * that or we use feedback from our quad encoders.
 */
void setMoveSpeed(int speed)
{
    if(speed > 50)
    {
        speed = 50;
    }
    else if(speed < 0)
    {
        speed = 0;
    }
    PWM_SetDutyCycle(PWM_LEFT_MOTOR,speed*10);
    PWM_SetDutyCycle(PWM_RIGHT_MOTOR,speed*10);
    
    return;
}

