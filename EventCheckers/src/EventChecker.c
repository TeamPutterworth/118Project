
/*******************************************************************************
 * MODULE #INCLUDE                                                             *
 ******************************************************************************/

#include "ES_Configure.h"
#include "EventChecker.h"
#include "ES_Events.h"
#include "serial.h"
#include "AD.h"
#include "BOARD.h"
#include "stdio.h"
#include "AD.h"
#include "pwm.h"
#include "LED.h"

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/
//#define DEBUG
/*******************************************************************************
 * EVENTCHECKER_TEST SPECIFIC CODE                                                             *
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/
/* Prototypes for private functions for this EventChecker. They should be functions
   relevant to the behavior of this particular event checker */

/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                    *
 ******************************************************************************/

/* Any private module level variable that you might need for keeping track of
   events would be placed here. Private variables should be STATIC so that they
   are limited in scope to this module. */

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

uint8_t trackWireSignal(void) {
    static ES_EventTyp_t PRESTATE = TRACK_WIRE_OFF;
    ES_EventTyp_t CURRENTTRACK;
    ES_Event thisEvent;
    uint8_t returnREAD = FALSE;
 
    uint8_t Whichtrack = readTrackWire();

    if (Whichtrack) {
        CURRENTTRACK = TRACK_WIRE_OFF;
        #ifdef DEBUG
        LED_SetBank(LED_BANK1,0x0);
        printf("\r\ntrack wire is on");
        #endif 
    } else {
        
        CURRENTTRACK = TRACK_WIRE_ON;
        #ifdef DEBUG
        LED_SetBank(LED_BANK1,0xF);
        printf("\r\ntrack wire is off");
        #endif
    }

    if (CURRENTTRACK != PRESTATE) {
        thisEvent.EventType = CURRENTTRACK;
        thisEvent.EventParam = Whichtrack;
        returnREAD = TRUE;
        PRESTATE = CURRENTTRACK;
       // PostGenericService(thisEvent);
    }
    return (returnREAD);

}

uint8_t beaconSignal(void) {
    static ES_EventTyp_t lastState = BEACON_OFF;
    ES_EventTyp_t curState;
    ES_Event thisEvent;
    uint8_t returnREAD = FALSE;
 
    uint8_t beaconVal = readBeaconDetector();

    if (beaconVal) {
        curState = BEACON_OFF;
        #ifdef DEBUG
        LED_SetBank(LED_BANK3,0x0);
        //printf("\r\ntrack wire is on");
        #endif
    } else {
        
        curState = BEACON_ON;
        #ifdef DEBUG
        //printf("\r\ntrack wire is off");
        LED_SetBank(LED_BANK3,0xF);
        #endif
    }

    if (curState != lastState) {
        thisEvent.EventType = BEACON_TRIGGERED;
        thisEvent.EventParam = !beaconVal;
        returnREAD = TRUE;
        lastState = curState;
       // PostGenericService(thisEvent);
    }
    return (returnREAD);
}
