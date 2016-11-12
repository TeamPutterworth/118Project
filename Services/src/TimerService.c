/*
 * File: TemplateService.h
 * Author: J. Edward Carryer
 * Modified: Gabriel H Elkaim
 *
 * Template file to set up a simple service to work with the Events and Services
 * Framework (ES_Framework) on the Uno32 for the CMPE-118/L class. Note that this file
 * will need to be modified to fit your exact needs, and most of the names will have
 * to be changed to match your code.
 *
 * This is provided as an example and a good place to start.
 *
 * Created on 23/Oct/2011
 * Updated on 13/Nov/2013
 */

/*******************************************************************************
 * MODULE #INCLUDE                                                             *
 ******************************************************************************/

#include "BOARD.h"
#include "AD.h"
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "TimerService.h"
#include <stdio.h>

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/
#define COUNTER_STEADY_STATE 3
#define LF_BUMPER 0x01
#define RF_BUMPER 0x02
#define LB_BUMPER 0x04
#define RB_BUMPER 0x08
#define BATTERY_DISCONNECT_THRESHOLD 175
#define TIMER_0_TICKS 5


/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/
/* Prototypes for private functions for this machine. They should be functions
   relevant to the behavior of this state machine */

/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                    *
 ******************************************************************************/
/* You will need MyPriority and maybe a state variable; you may need others
 * as well. */

static const char *eventName;
static ES_Event storedEvent;
static uint8_t MyPriority;

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

/**
 * @Function InitTimerService(uint8_t Priority)
 * @param Priority - internal variable to track which event queue to use
 * @return TRUE or FALSE
 * @brief This will get called by the framework at the beginning of the code
 *        execution. It will post an ES_INIT event to the appropriate event
 *        queue, which will be handled inside RunTimerService function. Remember
 *        to rename this to something appropriate.
 *        Returns TRUE if successful, FALSE otherwise
 * @author J. Edward Carryer, 2011.10.23 19:25 */
uint8_t InitTimerService(uint8_t Priority) {
    ES_Event ThisEvent;

    MyPriority = Priority;

    // in here you write your initialization code
    // this includes all hardware and software initialization
    // that needs to occur.

    // post the initial transition event
    ES_Timer_InitTimer(SYNC_SAMPLE_TIMER, TIMER_0_TICKS);
    ThisEvent.EventType = ES_INIT;
    if (ES_PostToService(MyPriority, ThisEvent) == TRUE) {
        return TRUE;
    } else {
        return FALSE;
    }
}

/**
 * @Function PostTimerService(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be posted to queue
 * @return TRUE or FALSE
 * @brief This function is a wrapper to the queue posting function, and its name
 *        will be used inside ES_Configure to point to which queue events should
 *        be posted to. Remember to rename to something appropriate.
 *        Returns TRUE if successful, FALSE otherwise
 * @author J. Edward Carryer, 2011.10.23 19:25 */
uint8_t PostTimerService(ES_Event ThisEvent) {
    return ES_PostToService(MyPriority, ThisEvent);
}

/**
 * @Function RunTimerService(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be responded.
 * @return Event - return event (type and param), in general should be ES_NO_EVENT
 * @brief This function is where you implement the whole of the service,
 *        as this is called any time a new event is passed to the event queue. 
 * @note Remember to rename to something appropriate.
 *       Returns ES_NO_EVENT if the event have been "consumed." 
 * @author J. Edward Carryer, 2011.10.23 19:25 */
ES_Event RunTimerService(ES_Event ThisEvent)
{
    ES_Event ReturnEvent;
    ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

    /********************************************
     in here you write your service code
     *******************************************/
//    static ES_EventTyp_t lastEvent = BATTERY_DISCONNECTED;
//    ES_EventTyp_t curEvent;
//    uint16_t batVoltage = AD_ReadADPin(BAT_VOLTAGE);
    static ES_EventTyp_t lastEvent [4] = {NOT_BUMPED,NOT_BUMPED,NOT_BUMPED,NOT_BUMPED};
    static int counter [4] = {0,0,0,0};
    static int lastBumpVal = 0;
    ES_EventTyp_t curEvent [4];
    ES_Event thisEvent [4];
    uint16_t curBump = Roach_ReadBumpers(); // read light sensor
    uint16_t debouncedBump = 0;
    int i;

    switch (ThisEvent.EventType) {
    case ES_INIT:
        // No hardware initialization or single time setups, those
        // go in the init function above.
        //
        // This section is used to reset service for some reason
        break;
    case ES_TIMERACTIVE:
    case ES_TIMERSTOPPED:
        break;
    case ES_TIMEOUT:
        ES_Timer_InitTimer(SIMPLE_SERVICE_TIMER, TIMER_0_TICKS);
//        if (batVoltage > BATTERY_DISCONNECT_THRESHOLD) { // is battery connected?
//            curEvent = BATTERY_CONNECTED;
//        } else {
//            curEvent = BATTERY_DISCONNECTED;
//        }
        if (curBump & LF_BUMPER) {
            counter[0]++;
        } else {
            counter[0] = 0;
        }
        if ((curBump & RF_BUMPER) >> 1) {
            counter[1]++;
        } else {
            counter[1] = 0;
        }

        if ((curBump & LB_BUMPER) >> 2) {
            counter[2]++;
        } else {
            counter[2] = 0;
        }

        if ((curBump & RB_BUMPER) >> 3) {
            counter[3]++;
        } else {
            counter[3] = 0;
        }
        
        for (i = 0; i < 4; i++) {
            if (counter[i] >= COUNTER_STEADY_STATE) {
                curEvent[i] = BUMPED;
            } else { 
                curEvent[i] = NOT_BUMPED;
            }
            
            if (curEvent[i] != lastEvent[i]) { // check for change from last time
                thisEvent[i].EventType = curEvent[i];
                thisEvent[i].EventParam = curBump;
                lastEvent[i] = curEvent[i]; // update history
#ifndef SIMPLESERVICE_TEST           // keep this as is for test harness
                PostGenericService(thisEvent[i]);
#else
                PostTimerService(thisEvent[i]);
#endif   
             }   
            //printf("\r\nCurrent bumper val: %d",debouncedBump);
        }
        
        break;
#ifdef SIMPLESERVICE_TEST     // keep this as is for test harness      
    default:
        printf("\r\nEvent: %s\tParam: 0x%X",
                EventNames[ThisEvent.EventType], ThisEvent.EventParam);
        break;
#endif
    }

    return ReturnEvent;
}

//    static ES_EventTyp_t lastEvent = NOT_BUMPED;
//    ES_EventTyp_t curEvent;
//    ES_Event thisEvent;
//    uint8_t returnVal = FALSE;
//    uint16_t curBump = Roach_ReadBumpers(); // read light sensor
//    //printf("Current light sensor val : %d \n",curLight);
//
//    
//    if(curBump != 0x0){
//        curEvent = BUMPED;
//    }
//    else {
//        curEvent = NOT_BUMPED;
//    }
//    
//    if (curEvent != lastEvent) { // check for change from last time
//        thisEvent.EventType = curEvent;
//        thisEvent.EventParam = curBump;
//        returnVal = TRUE;
//        lastEvent = curEvent; // update history
//#ifndef EVENTCHECKER_TEST           // keep this as is for test harness
//        PostTimerService(thisEvent);
//#else
//        SaveEvent(thisEvent);
//#endif   
//    }
//    return (returnVal); 
//ES_Event RunTimerService(ES_Event ThisEvent) {
//    int i;
//    int counter [4];
//    ES_Event ReturnEvent;
//    ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
//    static ES_EventTyp_t lastEvent = NOT_BUMPED;
//    static uint16_t lastBumpVal = 0;
//    ES_EventTyp_t curEvent;
//    ES_Event thisEvent;
//    uint16_t curBump, debouncedBump=0; // read light sensor
//
//    /********************************************
//     in here you write your service code
//     *******************************************/
//
//    switch (ThisEvent.EventType) {
//        case ES_INIT:
//            // No hardware initialization or single time setups, those
//            // go in the init function above.
//            //
//            // This section is used to reset service for some reason
//            break;
//
//        case ES_TIMERACTIVE:
//
//        case ES_TIMERSTOPPED:
//            break;
//
//        case ES_TIMEOUT:
//            ES_Timer_InitTimer(SIMPLE_SERVICE_TIMER, TIMER_0_TICKS);
//            curBump = Roach_ReadBumpers(); // read light sensor
//            if (curBump & LF_BUMPER) {
//                    counter[0]++;
//                } else {
//                    counter[0] = 0;
//                }
//
//                if ((curBump & RF_BUMPER) >> 1) {
//                    counter[1]++;
//                } else {
//                    counter[1] = 0;
//                }
//
//                if ((curBump & LB_BUMPER) >> 2) {
//                    counter[2]++;
//                } else {
//                    counter[2] = 0;
//                }
//
//                if ((curBump & RB_BUMPER) >> 3) {
//                    counter[3]++;
//                } else {
//                    counter[3] = 0;
//                }
//
//                for (i = 0; i < 4; i++) {
//                    if (counter[i] == COUNTER_STEADY_STATE) {
//                        debouncedBump = debouncedBump | (1 << i);
//                    }
//                }
//            //printf("Current light sensor val : %d \n",curLight);
//
//
//            if (debouncedBump != lastBumpVal && debouncedBump != 0x0) {
//                curEvent = BUMPED;
//            } else {
//                curEvent = NOT_BUMPED;
//            }
//
//            if (curEvent != lastEvent) { // check for change from last time
//                ReturnEvent.EventType = curEvent;
//                ReturnEvent.EventParam = curBump;
//                lastEvent = curEvent; // update history
//               
//#ifndef SIMPLESERVICE_TEST           // keep this as is for test harness
//                PostGenericService(ReturnEvent);
//#else
//                PostTimerService(ReturnEvent);
//#endif   
//                }
//                break;
//#ifdef SIMPLESERVICE_TEST     // keep this as is for test harness      
//                default:
//                printf("\r\nEvent: %s\tParam: 0x%X",
//                        EventNames[ThisEvent.EventType], ThisEvent.EventParam);
//#endif
//                break;
//        } // end switch
//    return ReturnEvent;
//}


/*******************************************************************************
 * PRIVATE FUNCTIONs                                                           *
 ******************************************************************************/

