#include <BOARD.h>
#include <xc.h>
#include <stdio.h>
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "LED.h"
#include "AD.h"
#include "pwm.h"
#include "motor.h"
#include "sensors.h"

#ifdef JANKY_TEST_HARNESS
void testTapeSensors(); 
void testBeaconDetector(); 
void testMotors(); 
void testServos();
void delay();
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
        switch(readBumpers()){
            case (0x01): // Front-Left Bumper
                testMotors();
                break;
            case (0x02): // Front-Right Bumper
                testTapeSensors();
                break;
            case (0x04): // Back-Left Bumper
                testBeaconDetector();
                break;
            case (0x08): // Back-Right Bumper
                testServos();
                break;
            default: // No bumpers or multiple bumpers
                break;
       }
    }
#endif
};

#ifdef JANKY_TEST_HARNESS
void testMotors()
{
    moveForward();
    setMoveSpeed(50);
    delay(1000000);
    setMoveSpeed(35);
    delay(1000000);
    setMoveSpeed(15);
    delay(1000000);
    stopMoving();
    moveBackward();
    delay(1000000);
    setMoveSpeed(50);
    delay(1000000);
    setMoveSpeed(35);
    delay(1000000);
    setMoveSpeed(15);
    delay(1000000);
    stopMoving();
    
    return;
}

void testTapeSensors() 
{
    return;
}

void testBeaconDetector()
{
    return;
}

void testServos()
{
    return;
}

void delay(int counter)
{
    int i = 0;
    
    for(i;i<=counter;i++)
    {
        ;
    }
    
    return;
}
#endif
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
