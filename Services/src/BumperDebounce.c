
// Jeremy Crowley
// CMPE 118 bumper debouncing service

/*******************************************************************************
 * MODULE #INCLUDE                                                             *
 ******************************************************************************/

#include "BOARD.h"
#include "LED.h"
#include "AD.h"
#include "IO_Ports.h"
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "BumperDebounce.h"
#include "sensors.h"
#include <stdio.h>

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/
#define DEBUG
#define LOW_TO_HIGH 0x7F
#define HIGH_TO_LOW 0x80
#define BUMPER_DEBOUNCE_TIMER_TICKS 5

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/
/* Prototypes for private functions for this machine. They should be functions
   relevant to the behavior of this state machine */

/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                    *
 ******************************************************************************/
static uint8_t MyPriority;
static uint16_t bumperPin[] = {FR_BUMPER, FL_BUMPER, B_BUMPER};
static uint16_t ledBanks[]={LED_BANK1,LED_BANK2,LED_BANK3};

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
uint8_t InitBumperDebounceService(uint8_t Priority) {
    ES_Event ThisEvent;

    MyPriority = Priority;

    ES_Timer_InitTimer(BUMPER_DEBOUNCE_TIMER,BUMPER_DEBOUNCE_TIMER_TICKS);
    
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
uint8_t PostBumperDebounceService(ES_Event ThisEvent) {
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
ES_Event RunBumperDebounceService(ES_Event ThisEvent)
{
    int i;
    static uint8_t bumperState[NUM_BUMPERS] = {0};
    static uint8_t pastBumperState[NUM_BUMPERS] = {0};
 
    ES_Event ReturnEvent;
    ReturnEvent.EventType = ES_NO_EVENT;
    ReturnEvent.EventParam = 0;
    
    // timer timeout event
    if(ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == BUMPER_DEBOUNCE_TIMER){
         
        for(i=0;i<NUM_BUMPERS;i++){
            // read in new state
            pastBumperState[i] = pastBumperState[i] << 1;
            bumperState[i] = (IO_PortsReadPort(PORTW) & bumperPin[i]) >> (3+i);   
            pastBumperState[i] |= bumperState[i];
            
            if(pastBumperState[i] == LOW_TO_HIGH){
                ReturnEvent.EventParam |= bumperPin[i];   
                PostTopLevelHSM(ReturnEvent);
                #ifdef DEBUG
                //LED_SetBank(ledBanks[i],0xF);
                printf("\r\nBumper %d bumped", i);
                #endif 
            }
        }

        // event has occurred if any bumper set the param
        if(ReturnEvent.EventParam){
                #ifdef DEBUG
                //LED_SetBank(ledBanks[i],0xF);
                printf("\r\nEventParam: %x", ReturnEvent.EventParam);
                #endif 
            ReturnEvent.EventType = BUMPED;
            PostTopLevelHSM(ReturnEvent);
        }
         
        // restart timer for this service
        ES_Timer_InitTimer(BUMPER_DEBOUNCE_TIMER,BUMPER_DEBOUNCE_TIMER_TICKS);
  
    }
    
    return ReturnEvent;
}