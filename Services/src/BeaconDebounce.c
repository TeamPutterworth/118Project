/*
 * File:   BeaconDebounce.c
 * Author: TeamPutterWorth
 * 
 * This service is responsible for debouncing the beacon and posting events
 * 
 */

/*******************************************************************************
 * MODULE #INCLUDE                                                             *
 ******************************************************************************/

#include "BOARD.h"
#include "LED.h"
#include "AD.h"
#include "IO_Ports.h"
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "sensors.h"
#include <stdio.h>

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/
//#define DEBUG
#ifdef DEBUG
#define TIMER_10_TICKS 500 // 2 ticks = 2 ms
#else
#define TIMER_10_TICKS 2 // 2 ticks = 2 ms
#endif
#define ON 0
#define OFF 1
#define BEACON_STEADY_STATE 5

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

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

/**
 * @Function InitBeaconDebounceService(uint8_t Priority)
 * @param Priority - internal variable to track which event queue to use
 * @return TRUE or FALSE
 * @brief This will get called by the framework at the beginning of the code
 *        execution. It will post an ES_INIT event to the appropriate event
 *        queue, which will be handled inside RunBeaconDebounceService function.
 *
 *        Returns TRUE if successful, FALSE otherwise
 */
uint8_t InitBeaconDebounceService(uint8_t Priority) {
    ES_Event ThisEvent;

    MyPriority = Priority;
    ES_Timer_InitTimer(BEACON_DEBOUNCE_TIMER, TIMER_10_TICKS);
    ThisEvent.EventType = ES_INIT;
    if (ES_PostToService(MyPriority, ThisEvent) == TRUE) {
        return TRUE;
    } else {
        return FALSE;
    }
}

/**
 * @Function PostBeaconDebounceService(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be posted to queue
 * @return TRUE or FALSE
 * @brief This function is a wrapper to the queue posting function, and its name
 *        will be used inside ES_Configure to point to which queue events should
 *        be posted to.
 *
 *        Returns TRUE if successful, FALSE otherwise
 */
uint8_t PostBeaconDebounceService(ES_Event ThisEvent) {
    return ES_PostToService(MyPriority, ThisEvent);
}

/**
 * @Function RunBeaconDebounceService(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be responded.
 * @return Event - return event (type and param), in general should be ES_NO_EVENT
 * @brief This service is responsible for synchronous sampling of a VISHAY TCRT5000.
 *        A sample is taken while the LED is on and while it is OFF and the difference
 *        is compared to a threshold to see if there was tape. 
 * 
 *       Returns ES_NO_EVENT if the event have been "consumed." 
 */
ES_Event RunBeaconDebounceService(ES_Event ThisEvent)
{
    ES_Event ReturnEvent;
    ES_Event PostEvent;
    ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
    uint8_t beaconVal;
    static uint8_t counterOn;
    static uint8_t counterOff;
    static ES_EventTyp_t lastState = BEACON_ON;
    ES_EventTyp_t curState;

    switch (ThisEvent.EventType) 
    {
        case ES_INIT:
            break;

        case ES_TIMERACTIVE:

        case ES_TIMERSTOPPED:
            break;

        case ES_TIMEOUT:
            ES_Timer_InitTimer(BEACON_DEBOUNCE_TIMER, TIMER_10_TICKS);
            beaconVal = readBeaconDetector();
            #ifdef DEBUG
            printf("\r\n\nFront TrackW: %d",!trackValF);
            printf("\r\nBack TrackW: %d",!trackVal);
            #endif

            if (beaconVal == ON)
            {
                counterOn++;
                counterOff = 0;
            }
            else
            {
                counterOff++;
                counterOn = 0;
            }

            if(counterOn > BEACON_STEADY_STATE)
            {
                curState = BEACON_ON;
            }
            else if(counterOff > BEACON_STEADY_STATE)
            {
                curState = BEACON_OFF;
            } else {
                curState = lastState;
            }
            // Choose this one to post as we read the front track wire first.
            if (curState != lastState)
            {
                lastState = curState;
                PostEvent.EventType = BEACON_TRIGGERED;
                if(curState == BEACON_ON)
                {
                    PostEvent.EventParam = 1;
                }else{
                    PostEvent.EventParam = 0;
                }

                #ifdef DEBUG
                //printf("\r\nFront TrackW: %d Back TrackW: %d",!trackValF,!trackVal);
                printf("\r\nParam: %d",PostEvent.EventParam);
                #endif
                PostTopLevelHSM(PostEvent);
            }
            break;

        default:
            printf("\r\nRecieved Event: %s with Param: 0x%X",
                    EventNames[ThisEvent.EventType], ThisEvent.EventParam);
            break;
    }
    return ReturnEvent;
}
