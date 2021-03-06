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
#define NORM_SPEED 25
#define MAX_VOLTAGE 9.9

void motorInit()
{
    IO_PortsSetPortOutputs(PORTY, DIR_LEFT_MOTOR|DIR_RIGHT_MOTOR);
    PWM_AddPins(PWM_LEFT_MOTOR|PWM_RIGHT_MOTOR);
}

void pivotTurnRight()
{
	motorForward(RIGHT);
    setMoveSpeed(NORM_SPEED);
	motorStop(LEFT);
}

void pivotTurnLeft()
{
	motorForward(LEFT);
    setMoveSpeed(NORM_SPEED);
	motorStop(RIGHT);
}

void pivotTurnRightBackward()
{
    motorStop(LEFT);
	motorBackward(RIGHT);
    setMotorSpeed(RIGHT, NORM_SPEED);
}

void pivotTurnLeftBackward()
{
    motorStop(RIGHT);
	motorBackward(LEFT);
    setMotorSpeed(LEFT, NORM_SPEED);
}


void tankTurnRight()
{
	motorForward(LEFT);
	motorBackward(RIGHT);
    setMoveSpeed(NORM_SPEED);
}

void tankTurnLeft()
{
	motorForward(RIGHT);
	motorBackward(LEFT);
    setMoveSpeed(NORM_SPEED);
}

void moveBackward()
{
	motorBackward(RIGHT);
	motorBackward(LEFT);
    setMoveSpeed(NORM_SPEED);
}

void moveForward()
{
	motorForward(LEFT);
	motorForward(RIGHT);
    setMoveSpeed(NORM_SPEED);
}

void stopMoving()
{
	setMoveSpeed(0);
}

void gradualTurnLeft(int difference)
{
    motorForward(LEFT);
    motorForward(RIGHT);
	setMotorSpeed(LEFT, NORM_SPEED - (difference/2));
	setMotorSpeed(RIGHT, NORM_SPEED + (difference/2));
}

void gradualTurnRight(int difference)
{
    motorForward(LEFT);
    motorForward(RIGHT);
	setMotorSpeed(LEFT, NORM_SPEED + (difference/2));
	setMotorSpeed(RIGHT, NORM_SPEED - (difference/2));
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
        IO_PortsWritePort(PORTY,IO_PortsReadPort(PORTY) | DIR_RIGHT_MOTOR); // Set Dir right motor = low (forward)
	}
	else
	{   
        IO_PortsWritePort(PORTY,IO_PortsReadPort(PORTY) & ~DIR_LEFT_MOTOR); // Set Dir left motor = high (forward)
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
		IO_PortsWritePort(PORTY,IO_PortsReadPort(PORTY) & ~DIR_RIGHT_MOTOR); // Set Dir right motor = high (backward)
	}
	else
	{
		IO_PortsWritePort(PORTY,IO_PortsReadPort(PORTY) | DIR_LEFT_MOTOR); // Set Dir left motor = low (backward)
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

void setMotorSpeed(int lr, int speed)
{
    uint16_t batVoltage = (AD_ReadADPin(BAT_VOLTAGE) * 33) / 1023; // read the battery voltage
	if(speed > 50)
    {
        speed = 50;
    }
    else if(speed < 0)
    {
        speed = 0;
    }
    
    speed = speed * 10; // Duty cycle is set in increments of 1/1000, rather than 1/100
    speed = speed * MAX_VOLTAGE/batVoltage;
 
    if (lr == RIGHT) 
	{
		PWM_SetDutyCycle(PWM_RIGHT_MOTOR, speed);
	}
	else
	{
		PWM_SetDutyCycle(PWM_LEFT_MOTOR, speed);
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
    uint16_t batVoltage = (AD_ReadADPin(BAT_VOLTAGE) * 33) / 1023; // read the battery voltage
    if(speed > 50)
    {
        speed = 50;
    }
    else if(speed < 0)
    {
        speed = 0;
    } 
    speed = speed * 10; // Duty cycle is set in increments of 1/1000, rather than 1/100
    speed = speed * MAX_VOLTAGE/batVoltage;
    PWM_SetDutyCycle(PWM_LEFT_MOTOR,speed);
    PWM_SetDutyCycle(PWM_RIGHT_MOTOR,speed);
    
    return;
}

