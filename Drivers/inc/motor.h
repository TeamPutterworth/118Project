/*
 * File:   motor.h
 * Author: jdgrant
 *
 * Motor file for helper functions which will be used to perform basic movement
 * and configure motor speed.
 * 
 */

#ifndef _MOTOR_H   
#define _MOTOR_H

// Init function adds nessescary PWM/DIR pins for each motor, does not init pwm or IO.
void motorInit(); 

// Tank turn right makes the left wheel go backwards and right wheel go forwards
void tankTurnRight();

// Tank turn left makes the right wheel go backwards and left wheel go forwards
void tankTurnLeft();

// To move backwards we make both wheels spin backwards
void moveBackward();

// To move forwards both wheels must spin forwards
void moveForward();

// This function sets speed (duty cycle) to 0
void stopMoving();

// This function is used to control direction of individual motor (l/r)
void motorForward(int lr);

// This function is used to control direction of individual motor (l/r)
void motorBackward(int lr);

// This function is used to set the speed of each motor (duty cycle).
// Both motors should have the same speed for easy FKM movements.
void setMoveSpeed(int speed);

// This function is used to pivot turn to the right
void pivotTurnRight();

// This function is used to pivot turn to the left
void pivotTurnLeft();

// This function can be used to stop a specific motor
void motorStop(int lr);


#endif /* _MOTOR_H */
