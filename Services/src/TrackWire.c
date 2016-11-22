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
#include "sensors.h"
#include <stdio.h>

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/
//#define DEBUG
#ifdef DEBUG
#define TIMER_5_TICKS 500 // 2 ticks = 2 ms
#else
#define TIMER_5_TICKS 8 // 2 ticks = 2 ms
#endif
#define BACK_TRACK_WIRE 1
#define FRONT_TRACK_WIRE 0
#define TRACK_WIRE_STEADY_STATE 2

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
 * @Function InitTrackWireService(uint8_t Priority)
 * @param Priority - internal variable to track which event queue to use
 * @return TRUE or FALSE
 * @brief This will get called by the framework at the beginning of the code
 *        execution. It will post an ES_INIT event to the appropriate event
 *        queue, which will be handled inside RunTrackWireService function.
 *
 *        Returns TRUE if successful, FALSE otherwise
 */
uint8_t InitTrackWireService(uint8_t Priority) {
    ES_Event ThisEvent;

    MyPriority = Priority;
    ES_Timer_InitTimer(TRACK_WIRE_TIMER, TIMER_5_TICKS);
    ThisEvent.EventType = ES_INIT;
    if (ES_PostToService(MyPriority, ThisEvent) == TRUE) {
        return TRUE;
    } else {
        return FALSE;
    }
}

/**
 * @Function PostTrackWireService(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be posted to queue
 * @return TRUE or FALSE
 * @brief This function is a wrapper to the queue posting function, and its name
 *        will be used inside ES_Configure to point to which queue events should
 *        be posted to.
 *
 *        Returns TRUE if successful, FALSE otherwise
 */
uint8_t PostTrackWireService(ES_Event ThisEvent) {
    return ES_PostToService(MyPriority, ThisEvent);
}

/**
 * @Function RunTrackWireService(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be responded.
 * @return Event - return event (type and param), in general should be ES_NO_EVENT
 * @brief This service is responsible for synchronous sampling of a VISHAY TCRT5000.
 *        A sample is taken while the LED is on and while it is OFF and the difference
 *        is compared to a threshold to see if there was tape. 
 * 
 *       Returns ES_NO_EVENT if the event have been "consumed." 
 */
ES_Event RunTrackWireService(ES_Event ThisEvent)
{
    ES_Event ReturnEvent;
    ES_Event PostEvent;
    ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
    static uint8_t state;
    uint8_t trackVal;
    static uint8_t counter[2];
    static uint8_t trackValF;
    static ES_EventTyp_t lastState [] = {TRACK_WIRE_OFF,TRACK_WIRE_OFF};
    static ES_EventTyp_t curStateF;
    ES_EventTyp_t curState;

    switch (ThisEvent.EventType) 
    {
        case ES_INIT:
            break;

        case ES_TIMERACTIVE:

        case ES_TIMERSTOPPED:
            break;

        case ES_TIMEOUT:
            ES_Timer_InitTimer(TRACK_WIRE_TIMER, TIMER_5_TICKS);
            
            if (state == FRONT_TRACK_WIRE) // Read the front track wire val, set sel for back track
            {
                state = BACK_TRACK_WIRE;
                trackValF = readTrackWire();
                
                #ifdef DEBUG
                //printf("\r\nFront TrackW: %d",!trackValF);
                #endif
                
                if (trackValF)
                {
                    counter[0] = 0;
                }
                else
                {
                    counter[0]++;
                }
                
                if(counter[0] > TRACK_WIRE_STEADY_STATE)
                {
                    curStateF = TRACK_WIRE_ON;
                }
                else
                {
                    curStateF = TRACK_WIRE_OFF;
                }
                muxSelTrackWire(BACK_TRACK_WIRE);
            }
            else if (state == BACK_TRACK_WIRE) // Read the back track wire val, set sel for front track
            {
                state = FRONT_TRACK_WIRE;
                trackVal = readTrackWire();
                
                #ifdef DEBUG
                printf("\r\n\nFront TrackW: %d",!trackValF);
                printf("\r\nBack TrackW: %d",!trackVal);
                #endif
              
                if (trackVal)
                {
                    counter[1] = 0;
                }
                else
                {
                    counter[1]++;
                }
                
                if(counter[1] > TRACK_WIRE_STEADY_STATE)
                {
                    curState = TRACK_WIRE_ON;
                }
                else
                {
                    curState = TRACK_WIRE_OFF;
                }
                muxSelTrackWire(FRONT_TRACK_WIRE);
                // Choose this one to post as we read the front track wire first.
                if (curState != lastState[1] || curStateF != lastState[0])
                {
                    lastState[0] = curStateF;
                    lastState[1] = curState;
                    PostEvent.EventType = TW_TRIGGERED;
                    PostEvent.EventParam = 0;
                    if(curState == TRACK_WIRE_ON)
                    {
                        PostEvent.EventParam |= 2;
                    }
                    else             
                    if(curStateF == TRACK_WIRE_ON)
                    {
                        PostEvent.EventParam |= 1;
                    }
                    #ifdef DEBUG
                    //printf("\r\nFront TrackW: %d Back TrackW: %d",!trackValF,!trackVal);
                    printf("\r\nParam: %d",PostEvent.EventParam);
                    #endif
                    PostTopLevelHSM(PostEvent);
                }
            }
            break;

        default:
            printf("\r\nRecieved Event: %s with Param: 0x%X",
                    EventNames[ThisEvent.EventType], ThisEvent.EventParam);
            break;
    }
    return ReturnEvent;
}
