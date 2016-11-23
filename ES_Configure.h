/****************************************************************************
 Module
     ES_Configure.h
 Description
     This file contains macro definitions that are edited by the user to
     adapt the Events and Services framework to a particular application.
 Notes
     
 History
 When           Who     What/Why
 -------------- ---     --------
 01/15/12 10:03 jec      started coding
 *****************************************************************************/

#ifndef CONFIGURE_H
#define CONFIGURE_H



//defines for keyboard input
//#define USE_KEYBOARD_INPUT
//What State machine are we testing
//#define POSTFUNCTION_FOR_KEYBOARD_INPUT PostTopLevelHSM

//define for TattleTale
#define USE_TATTLETALE

//uncomment to supress the entry and exit events
//#define SUPPRESS_EXIT_ENTRY_IN_TATTLE

/****************************************************************************/
// Name/define the events of interest
// Universal events occupy the lowest entries, followed by user-defined events

/****************************************************************************/
typedef enum {
    ES_NO_EVENT, ES_ERROR, /* used to indicate an error from the service */
    ES_INIT, /* used to transition from initial pseudo-state */
    ES_ENTRY, /* used to enter a state*/
    ES_EXIT, /* used to exit a state*/
    ES_KEYINPUT, /* used to signify a key has been pressed*/
    ES_LISTEVENTS, /* used to list events in keyboard input, does not get posted to fsm*/
    ES_TIMEOUT, /* signals that the timer has expired */
    ES_TIMERACTIVE, /* signals that a timer has become active */
    ES_TIMERSTOPPED, /* signals that a timer has stopped*/
    /* User-defined events start here */
    OFF_TAPE,
    ON_TAPE,
    TAPE_TRIGGERED,
    TW_TRIGGERED,
    TRACK_WIRE_ON,
    TRACK_WIRE_OFF,
    BEACON_OFF,  
    BEACON_ON,
    BUMPED,
    UNLOADED,
    BEACON_TRIGGERED,
    BATTERY_CONNECTED,
    BATTERY_DISCONNECTED,
    NUMBEROFEVENTS,
} ES_EventTyp_t;

static const char *EventNames[] = {
	"ES_NO_EVENT",
	"ES_ERROR",
	"ES_INIT",
	"ES_ENTRY",
	"ES_EXIT",
	"ES_KEYINPUT",
	"ES_LISTEVENTS",
	"ES_TIMEOUT",
	"ES_TIMERACTIVE",
	"ES_TIMERSTOPPED",
	"OFF_TAPE",
	"ON_TAPE",
	"TAPE_TRIGGERED",
	"TW_TRIGGERED",
	"TRACK_WIRE_ON",
	"TRACK_WIRE_OFF",
	"BEACON_OFF",
	"BEACON_ON",
	"BUMPED",
	"UNLOADED",
	"BEACON_TRIGGERED",
	"BATTERY_CONNECTED",
	"BATTERY_DISCONNECTED",
	"NUMBEROFEVENTS",
};




/****************************************************************************/
// This are the name of the Event checking function header file. 
// Our events are now legacy code and not actually run.
#define EVENT_CHECK_HEADER "EventChecker.h" 
/****************************************************************************/
// This is the list of event checking functions
#define EVENT_CHECK_LIST 

/****************************************************************************/
// These are the definitions for the post functions to be executed when the
// corresponding timer expires. All 16 must be defined. If you are not using
// a timers, then you can use TIMER_UNUSED
#define TIMER_UNUSED ((pPostFunc)0)
#define TIMER0_RESP_FUNC PostSyncSamplingService
#define TIMER1_RESP_FUNC PostTopLevelHSM
#define TIMER2_RESP_FUNC PostTopLevelHSM
#define TIMER3_RESP_FUNC PostTopLevelHSM
#define TIMER4_RESP_FUNC PostBumperDebounceService
#define TIMER5_RESP_FUNC PostTrackWireService
#define TIMER6_RESP_FUNC PostTopLevelHSM
#define TIMER7_RESP_FUNC PostTopLevelHSM
#define TIMER8_RESP_FUNC PostTopLevelHSM
#define TIMER9_RESP_FUNC PostTopLevelHSM
#define TIMER10_RESP_FUNC PostBeaconDebounceService
#define TIMER11_RESP_FUNC PostTopLevelHSM
#define TIMER12_RESP_FUNC TIMER_UNUSED
#define TIMER13_RESP_FUNC TIMER_UNUSED
#define TIMER14_RESP_FUNC TIMER_UNUSED
#define TIMER15_RESP_FUNC TIMER_UNUSED

/****************************************************************************/
// Give the timer numbers symbolc names to make it easier to move them
// to different timers if the need arises. Keep these definitons close to the
// definitions for the response functions to make it easire to check that
// the timer number matches where the timer event will be routed

// Simple service timer is only for keyboard inputs
#define SIMPLE_SERVICE_TIMER 0

#define SYNC_SAMPLE_TIMER 0 /*make sure this is enabled above and posting to the correct state machine*/
#define SHORT_HSM_TIMER 1
#define MEDIUM_HSM_TIMER 2
#define LONG_HSM_TIMER 3
#define BUMPER_DEBOUNCE_TIMER 4
#define TRACK_WIRE_TIMER 5
#define TIMER_22 9
#define TIMER_45 6
#define TIMER_90 7
#define TIMER_180 8
#define BEACON_DEBOUNCE_TIMER 10
#define TIMER_360 11

#define SHORT_TIMER_TICKS 50
#define MEDIUM_TIMER_TICKS 250 
#define LONG_TIMER_TICKS 1000

#define TIMER_45_TICKS 685
#define TIMER_22_TICKS TIMER_45_TICKS/2
#define TIMER_90_TICKS 2*TIMER_45_TICKS
#define TIMER_180_TICKS 4*TIMER_45_TICKS
#define TIMER_360_TICKS 8*TIMER_45_TICKS

/****************************************************************************/
// The maximum number of services sets an upper bound on the number of 
// services that the framework will handle. Reasonable values are 8 and 16
// HOWEVER: at this time only a value of 8 is supported.
#define MAX_NUM_SERVICES 8

/****************************************************************************/
// This macro determines that nuber of services that are *actually* used in
// a particular application. It will vary in value from 1 to MAX_NUM_SERVICES
#define NUM_SERVICES 6

/****************************************************************************/
// These are the definitions for Service 0, the lowest priority service
// every Events and Services application must have a Service 0. Further 
// services are added in numeric sequence (1,2,3,...) with increasing 
// priorities
// the header file with the public fuction prototypes
#define SERV_0_HEADER "ES_KeyboardInput.h"
// the name of the Init function
#define SERV_0_INIT InitKeyboardInput
// the name of the run function
#define SERV_0_RUN RunKeyboardInput
// How big should this service's Queue be?
#define SERV_0_QUEUE_SIZE 9

/****************************************************************************/
// These are the definitions for Service 1
#if NUM_SERVICES > 1
#define SERV_1_HEADER "SyncSampling.h"
// the name of the Init function
#define SERV_1_INIT InitSyncSamplingService
// the name of the run function
#define SERV_1_RUN RunSyncSamplingService
// How big should this services Queue be?
#define SERV_1_QUEUE_SIZE 3
#endif

// These are the definitions for Service 2
#if NUM_SERVICES > 2
// the header file with the public fuction prototypes
#define SERV_2_HEADER "TopLevelHSM.h"
// the name of the Init function
#define SERV_2_INIT InitTopLevelHSM
// the name of the run function
#define SERV_2_RUN RunTopLevelHSM
// How big should this services Queue be?
#define SERV_2_QUEUE_SIZE 3
#endif



/****************************************************************************/
// These are the definitions for Service 3
#if NUM_SERVICES > 3
// the header file with the public fuction prototypes
#define SERV_3_HEADER "BumperDebounce.h"
// the name of the Init function
#define SERV_3_INIT InitBumperDebounceService
// the name of the run function
#define SERV_3_RUN RunBumperDebounceService
// How big should this services Queue be?
#define SERV_3_QUEUE_SIZE 3
#endif

/****************************************************************************/
// These are the definitions for Service 4
#if NUM_SERVICES > 4
// the header file with the public fuction prototypes
#define SERV_4_HEADER "TrackWire.h"
// the name of the Init function
#define SERV_4_INIT InitTrackWireService
// the name of the run function
#define SERV_4_RUN RunTrackWireService
// How big should this services Queue be?
#define SERV_4_QUEUE_SIZE 3
#endif

/****************************************************************************/
// These are the definitions for Service 5
#if NUM_SERVICES > 5
// the header file with the public fuction prototypes
#define SERV_5_HEADER "BeaconDebounce.h"
// the name of the Init function
#define SERV_5_INIT InitBeaconDebounceService
// the name of the run function
#define SERV_5_RUN RunBeaconDebounceService
// How big should this services Queue be?
#define SERV_5_QUEUE_SIZE 3
#endif

/****************************************************************************/
// These are the definitions for Service 6
#if NUM_SERVICES > 6
// the header file with the public fuction prototypes
#define SERV_6_HEADER "TestService.h"
// the name of the Init function
#define SERV_6_INIT TestServiceInit
// the name of the run function
#define SERV_6_RUN TestServiceRun
// How big should this services Queue be?
#define SERV_6_QUEUE_SIZE 3
#endif

/****************************************************************************/
// These are the definitions for Service 7
#if NUM_SERVICES > 7
// the header file with the public fuction prototypes
#define SERV_7_HEADER "TestService.h"
// the name of the Init function
#define SERV_7_INIT TestServiceInit
// the name of the run function
#define SERV_7_RUN TestServiceRun
// How big should this services Queue be?
#define SERV_7_QUEUE_SIZE 3
#endif

/****************************************************************************/
// the name of the posting function that you want executed when a new 
// keystroke is detected.
// The default initialization distributes keystrokes to all state machines
#define POST_KEY_FUNC ES_PostAll



/****************************************************************************/
// These are the definitions for the Distribution lists. Each definition
// should be a comma seperated list of post functions to indicate which
// services are on that distribution list.
#define NUM_DIST_LISTS 0
#if NUM_DIST_LISTS > 0 
#define DIST_LIST0 PostTemplateFSM
#endif
#if NUM_DIST_LISTS > 1 
#define DIST_LIST1 PostTemplateFSM
#endif
#if NUM_DIST_LISTS > 2 
#define DIST_LIST2 PostTemplateFSM
#endif
#if NUM_DIST_LISTS > 3 
#define DIST_LIST3 PostTemplateFSM
#endif
#if NUM_DIST_LISTS > 4 
#define DIST_LIST4 PostTemplateFSM
#endif
#if NUM_DIST_LISTS > 5 
#define DIST_LIST5 PostTemplateFSM
#endif
#if NUM_DIST_LISTS > 6 
#define DIST_LIST6 PostTemplateFSM
#endif
#if NUM_DIST_LISTS > 7 
#define DIST_LIST7 PostTemplateFSM
#endif



#endif /* CONFIGURE_H */