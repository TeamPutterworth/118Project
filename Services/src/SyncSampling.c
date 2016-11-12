// Joseph Grant
// Midterm Problem 8
// 11/1/16

/*******************************************************************************
 * MODULE #INCLUDE                                                             *
 ******************************************************************************/

#include "BOARD.h"
#include "LED.h"
#include "AD.h"
#include "IO_Ports.h"
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "TimerService.h"
#include "motor.h"
#include <stdio.h>

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/
//#define DEBUG
//#define MOTOR_TEST 
#ifdef DEBUG
#define TIMER_0_TICKS 500 // 2 ticks = 2 ms
#else
#define TIMER_0_TICKS 2 // 2 ticks = 2 ms
#endif
#define ON 1
#define HI_THRESHOLD 150
#define LO_THRESHOLD 100
#define NUM_LEDS 5

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/
/* Prototypes for private functions for this machine. They should be functions
   relevant to the behavior of this state machine */

/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                    *
 ******************************************************************************/

static const char *eventName;
static ES_Event storedEvent;
static uint8_t MyPriority;
static unsigned int adPins[]={AD_PORTV3,AD_PORTV4,AD_PORTV5,AD_PORTV6,AD_PORTV7};
static uint16_t ledPins[]={PIN3,PIN4,PIN5,PIN7,PIN10};
static uint16_t ledBanks[]={LED_BANK1,LED_BANK1,LED_BANK2,LED_BANK2,LED_BANK3};

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

/**
 * @Function InitSyncSamplingService(uint8_t Priority)
 * @param Priority - internal variable to track which event queue to use
 * @return TRUE or FALSE
 * @brief This will get called by the framework at the beginning of the code
 *        execution. It will post an ES_INIT event to the appropriate event
 *        queue, which will be handled inside RunSyncSamplingService function.
 *
 *        Returns TRUE if successful, FALSE otherwise
 */
uint8_t InitSyncSamplingService(uint8_t Priority) {
    ES_Event ThisEvent;

    MyPriority = Priority;

    // TODO : Move this init stuff to their proper init functions and then call the init
    // functions in main.
    
    IO_PortsSetPortOutputs(PORTZ, PIN3|PIN4|PIN5|PIN7|PIN10);
    AD_AddPins(AD_PORTV3|AD_PORTV4|AD_PORTV5|AD_PORTV6|AD_PORTV7);
#ifdef MOTOR_TEST
    moveForward();
    setMoveSpeed(50);
#endif 
    // post the initial transition event
    IO_PortsWritePort(PORTZ,IO_PortsReadPort(PORTZ) | PIN3); // This Sets PIN7 (the LED) high for the first sample
    IO_PortsWritePort(PORTZ,IO_PortsReadPort(PORTZ) | PIN4); // This Sets PIN7 (the LED) high for the first sample
    IO_PortsWritePort(PORTZ,IO_PortsReadPort(PORTZ) | PIN5); // This Sets PIN7 (the LED) high for the first sample
    IO_PortsWritePort(PORTZ,IO_PortsReadPort(PORTZ) | PIN7); // This Sets PIN7 (the LED) high for the first sample
    IO_PortsWritePort(PORTZ,IO_PortsReadPort(PORTZ) | PIN10); // This Sets PIN7 (the LED) high for the first sample
    ES_Timer_InitTimer(SYNC_SAMPLE_TIMER, TIMER_0_TICKS);
    ThisEvent.EventType = ES_INIT;
    if (ES_PostToService(MyPriority, ThisEvent) == TRUE) {
        return TRUE;
    } else {
        return FALSE;
    }
}

/**
 * @Function PostSyncSamplingService(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be posted to queue
 * @return TRUE or FALSE
 * @brief This function is a wrapper to the queue posting function, and its name
 *        will be used inside ES_Configure to point to which queue events should
 *        be posted to.
 *
 *        Returns TRUE if successful, FALSE otherwise
 */
uint8_t PostSyncSamplingService(ES_Event ThisEvent) {
    return ES_PostToService(MyPriority, ThisEvent);
}

/**
 * @Function RunSyncSamplingService(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be responded.
 * @return Event - return event (type and param), in general should be ES_NO_EVENT
 * @brief This service is responsible for synchronous sampling of a VISHAY TCRT5000.
 *        A sample is taken while the LED is on and while it is OFF and the difference
 *        is compared to a threshold to see if there was tape. 
 * 
 *       Returns ES_NO_EVENT if the event have been "consumed." 
 */
ES_Event RunSyncSamplingService(ES_Event ThisEvent)
{
    int i;
    uint8_t tapeTriggered = FALSE;
    ES_Event ReturnEvent;
    ES_Event PostEvent;
    ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
    static uint8_t curLEDState = ON; // LED Starts as on (in init function).
    static int16_t adcValOn[NUM_LEDS], adcValOff[NUM_LEDS], adcDiff[NUM_LEDS];
    static ES_EventTyp_t lastEvent [NUM_LEDS] = {OFF_TAPE,OFF_TAPE,OFF_TAPE,OFF_TAPE};
    ES_EventTyp_t curEvent [4];

    switch (ThisEvent.EventType) 
    {
        case ES_INIT:
            break;

        case ES_TIMERACTIVE:

        case ES_TIMERSTOPPED:
            break;

        case ES_TIMEOUT:
            ES_Timer_InitTimer(SYNC_SAMPLE_TIMER, TIMER_0_TICKS);

            // We always either turn the LED(s) on or off, it is simple!
            if (curLEDState == ON)
            {   
                for(i = 0;i < NUM_LEDS;i++)
                {
                    adcValOn[i] = AD_ReadADPin(adPins[i]); // Read the LED while it has been on for 2 ms 
                    #ifdef DEBUG
                    printf("\r\nadcValOn[%d]: %d",i,adcValOn[i]);
                    #endif
                    IO_PortsWritePort(PORTZ,IO_PortsReadPort(PORTZ) & ~ledPins[i]); // This Sets PINi (the LED) low  
                }
                curLEDState = FALSE;
            } else {
                curLEDState = ON;
                for(i = 0;i < NUM_LEDS;i++)
                {
                    adcValOff[i] = AD_ReadADPin(adPins[i]); // Read the LED after it has been off for 2 ms
                    #ifdef DEBUG
                    printf("\r\nadcValOff[%d]: %d",i,adcValOff[i]);
                    #endif
                    IO_PortsWritePort(PORTZ,IO_PortsReadPort(PORTZ) | ledPins[i]); // This Sets PIN7 (the LED) high

                    adcDiff[i] = adcValOn[i] - adcValOff[i];
                    #ifdef DEBUG
                    printf("\r\nadcDiff[%d]: %d",i,adcDiff[i]);
                    #endif
                    // Do hystersis check
                    if (adcDiff[i] > HI_THRESHOLD) {
                        if ((i % 2) == 0)
                        {
                            LED_SetBank(ledBanks[i],LED_GetBank(ledBanks[i]) | 0x3);
                        }
                        if ((i % 2) != 0)
                        {
                           LED_SetBank(ledBanks[i],LED_GetBank(ledBanks[i]) | 0xC); 
                        }
                        curEvent[i] = OFF_TAPE;
                    }
                    else if (adcDiff[i] < LO_THRESHOLD)
                    {
                        if ((i % 2) == 0)
                        {
                            LED_SetBank(ledBanks[i],LED_GetBank(ledBanks[i]) & ~0x3);
                        }
                        if ((i % 2) != 0)
                        {
                           LED_SetBank(ledBanks[i],LED_GetBank(ledBanks[i]) & ~0xC); 
                        }
                        curEvent[i] = ON_TAPE;
                    }
                    // if we get into this condition, a tape sensor was triggered either on or off
                    // this is where we can post events to our fsm
                    if (curEvent[i] != lastEvent[i])
                    {
                        tapeTriggered = TRUE;
                        #ifdef MOTOR_TEST
                        if (curEvent[i] == ON_TAPE)
                        {
                            stopMoving();
                        }
                        else
                        {
                            setMoveSpeed(50);
                        }
                        #endif
                    }
                    lastEvent[i] = curEvent[i];
                }
                // On further thought, we decided posting one event with curLevel of each sensor is a better idea
                if (tapeTriggered)
                {   
                    PostEvent.EventParam = 0;
                    PostEvent.EventType = TAPE_TRIGGERED;
                    for(i = 0;i < NUM_LEDS;i++)
                    {
                        if(curEvent[i] == ON_TAPE)
                        {
                            PostEvent.EventParam = PostEvent.EventParam | (1 << i);
                        }
                        else
                        {
                            PostEvent.EventParam = PostEvent.EventParam & ~(1 << i);
                        }                
                    }
                    PostTopLevelHSM(PostEvent);
                    tapeTriggered = FALSE;      
                }
            }
            break;

        default:
            printf("\r\nRecieved Event: %s\twith Param: 0x%X",
                    EventNames[ThisEvent.EventType], ThisEvent.EventParam);
            break;
    }
    return ReturnEvent;
}