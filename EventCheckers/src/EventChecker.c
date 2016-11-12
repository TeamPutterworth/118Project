
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

uint8_t Track_Wire_Signal(void) {
    static ES_EventTyp_t PRESTATE = TRACK_WIRE_OFF;
    ES_EventTyp_t CURRENTTRACK;
    ES_Event thisEvent;
    uint8_t returnREAD = FALSE;
 
    uint8_t Whichtrack = readTrackWire();

    if (Whichtrack) {
        CURRENTTRACK = TRACK_WIRE_OFF;
        LED_SetBank(LED_BANK1,0x0);
        //printf("\r\ntrack wire is on");
    } else {
        
        CURRENTTRACK = TRACK_WIRE_ON;
        //printf("\r\ntrack wire is off");
        LED_SetBank(LED_BANK1,0xF);
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

uint8_t Beacon_Signal(void) {
    static ES_EventTyp_t PRESTATE = BEACON_OFF;
    ES_EventTyp_t CURRENTBEACON;
    ES_Event thisEvent;
    uint8_t returnREAD = FALSE;
 
    uint8_t Whichbeacon = readBeaconDetector();

    if (Whichbeacon) {
        CURRENTBEACON = BEACON_OFF;
        LED_SetBank(LED_BANK3,0x0);
        //printf("\r\ntrack wire is on");
    } else {
        
        CURRENTBEACON = BEACON_ON;
        //printf("\r\ntrack wire is off");
        LED_SetBank(LED_BANK3,0xF);
    }

    if (CURRENTBEACON != PRESTATE) {
        thisEvent.EventType = CURRENTBEACON;
        thisEvent.EventParam = Whichbeacon;
        returnREAD = TRUE;
        PRESTATE = CURRENTBEACON;
       // PostGenericService(thisEvent);
    }
    return (returnREAD);

}