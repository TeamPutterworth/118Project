#include <BOARD.h>
#include <xc.h>
#include <stdio.h>
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "LED.h"
#include "AD.h"
#include "RC_Servo.h"
#include "pwm.h"
#include "motor.h"
#include "sensors.h"

//#define JANKY_TEST_HARNESS
#ifdef JANKY_TEST_HARNESS

void delay(int counter)
{
    int i = 0;
    
    for(i;i<=counter;i++)
    {
        ;
    }
    
    return;
}

void testMotors()
{
    tankTurnRight();
    setMoveSpeed(50);
    delay(1000000);
    tankTurnLeft();
    setMoveSpeed(35);
    delay(1000000);
    moveBackward();
    setMoveSpeed(15);
    delay(1000000);
    stopMoving();
    delay(100000);
    moveForward();
    setMoveSpeed(50);
    delay(1000000);
    pivotTurnRight();
    setMoveSpeed(35);
    delay(1000000);
    pivotTurnLeft();
    setMoveSpeed(15);
    delay(1000000);
    stopMoving();
    
    return;
}

void testTrackWire() 
{
    // Set to pass through track wire 0
    muxSelTrackWire(0x0);
    delay(100); // Propagation time is very quick (ns), this is probably overkill
    if (readTrackWire())
    {
        LED_SetBank(LED_BANK1,0xF);
    }
    
    // Set to pass through track wire 1
    muxSelTrackWire(0x1);
    delay(100); // Propagation time is very quick (ns), this is probably overkill
    if (readTrackWire())
    {
        LED_SetBank(LED_BANK2,0xF);
    }
    
    delay(10000);
    LED_SetBank(LED_BANK1,0x0);
    LED_SetBank(LED_BANK2,0x0);
    LED_SetBank(LED_BANK3,0x0);
    return;
}

void testBeaconDetector()
{
    if (readBeaconDetector())
    {
        LED_SetBank(LED_BANK1,0xF);
        LED_SetBank(LED_BANK2,0xF);
        LED_SetBank(LED_BANK3,0xF);
    }
   
    delay(1000000);
    LED_SetBank(LED_BANK1,0x0);
    LED_SetBank(LED_BANK2,0x0);
    LED_SetBank(LED_BANK3,0x0);
    return;
}

void testServos()
{
    int i;
    for (i=1000;i<2000;i+=1)
    {
        RC_SetPulseTime(RC_SERVO_UNLOADING,i);
        RC_SetPulseTime(RC_SERVO_BRIDGE,i);
        delay(1000);
    }
    delay(100000);
    for (i=2000;i>1000;i-=1)
    {
        RC_SetPulseTime(RC_SERVO_UNLOADING,i);
        RC_SetPulseTime(RC_SERVO_BRIDGE,i);
        delay(1000);
    }
    delay(100000);
    return;
}
#endif

void main(void)
{
#ifndef JANKY_TEST_HARNESS
    ES_Return_t ErrorType;
#endif
    BOARD_Init();
    
    // Your hardware initialization function calls go here
    LED_Init();
    AD_Init();
    PWM_Init();
    RC_Init();
    motorInit();
    sensorsInit();
    
    LED_AddBanks(LED_BANK1|LED_BANK2|LED_BANK3);
    LED_SetBank(LED_BANK1,0x0);
    LED_SetBank(LED_BANK2,0x0);
    LED_SetBank(LED_BANK3,0x0);
    
#ifndef JANKY_TEST_HARNESS
    printf("Starting ES Framework\r\n");
    printf("using the 2nd Generation Events & Services Framework\r\n");
    
    // now initialize the Events and Services Framework and start it running
    ErrorType = ES_Initialize();
    if (ErrorType == Success) {
        ErrorType = ES_Run();

    }
    //if we got to here, there was an error
    switch (ErrorType) {
    case FailedPointer:
        printf("Failed on NULL pointer");
        break;
    case FailedInit:
        printf("Failed Initialization");
        break;
    default:
        printf("Other Failure: %d", ErrorType);
        break;
    }
    for (;;)
        ;
#else
    while (1) {
        testServos();
        /*
        switch(readBumpers()){
            case (0x01): // Front-Right Bumper
                testMotors();
                break;
            case (0x02): // Front-Left Bumper
                testTrackWire();
                break;
            case (0x03): // Both Front Bumpers!
                testBeaconDetector();
                break;
            case (0x04): // Plunger Bumper
                testServos();
                break;
            
            default: // No bumpers or multiple bumpers
                break;
       }
       */
    }
#endif
};
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
