/*
 * File:   TopLevelHSM.h
 * Author: jdgrant
 *
 * This file includes the top level of our hierarchal state machine. At this level
 * the robot performs the basic logic of finding loading stations, loading ammo, 
 * and finding targets to deposit the ammo. This basic logic is looped.
 * 
 */

/*******************************************************************************
 * MODULE #INCLUDE                                                             *
 ******************************************************************************/

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "BOARD.h"
#include "TopLevelHSM.h"
#include "sensors.h"

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/
#define PLUNGER_BUMPER 0x4


typedef enum {
    InitPState,
    AmmoSearch,
    AmmoLoad,
    FirstTargetSearch,
    FirstTargetUnload,
    SecondTargetSearch,
    SecondTargetUnload,
} TopLevelHSMState_t;

static const char *StateNames[] = {
	"InitPState",
	"AmmoSearch",
	"AmmoLoad",
	"FirstTargetSearch",
	"FirstTargetUnload",
	"SecondTargetSearch",
	"SecondTargetUnload",
};


/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/
/* Prototypes for private functions for this machine. They should be functions
   relevant to the behavior of this state machine
   Example: char RunAway(uint_8 seconds);*/
/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                            *
 ******************************************************************************/
/* You will need MyPriority and the state variable; you may need others as well.
 * The type of state variable should match that of enum in header file. */

static TopLevelHSMState_t CurrentState = InitPState; // <- change enum name to match ENUM
static uint8_t MyPriority;


/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

/**
 * @Function InitTopLevelHSM(uint8_t Priority)
 * @param Priority - internal variable to track which event queue to use
 * @return TRUE or FALSE
 * @brief This will get called by the framework at the beginning of the code
 *        execution. It will post an ES_INIT event to the appropriate event
 *        queue, which will be handled inside RunTopLevelHSM function.
 */
uint8_t InitTopLevelHSM(uint8_t Priority)
{
    MyPriority = Priority;
    // put us into the Initial PseudoState
    CurrentState = InitPState;
    // post the initial transition event
    if (ES_PostToService(MyPriority, INIT_EVENT) == TRUE) {
        return TRUE;
    } else {
        return FALSE;
    }
}

/**
 * @Function PostTopLevelHSM(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be posted to queue
 * @return TRUE or FALSE
 * @brief This function is a wrapper to the queue posting function.
 */
uint8_t PostTopLevelHSM(ES_Event ThisEvent)
{
    return ES_PostToService(MyPriority, ThisEvent);
}

/**
 * @Function RunTopLevelHSM(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be responded.
 * @return Event - return event (type and param), in general should be ES_NO_EVENT
 * @brief This is the implementation for the top level of our state machine and 
 *        it contains the basic logic for our autonomous robot to meet the minimum
 *        specs of the competition.
 */
ES_Event RunTopLevelHSM(ES_Event ThisEvent)
{
    uint8_t makeTransition = FALSE; // use to flag transition
    TopLevelHSMState_t nextState; // <- change type to correct enum

    ES_Tattle(); // trace call stack

    switch (CurrentState) {
    case InitPState: // If current state is initial Pseudo State
        if (ThisEvent.EventType == ES_INIT)// only respond to ES_Init
        {
            // this is where you would put any actions associated with the
            // transition from the initial pseudo-state into the actual
            // initial state
            // Initialize all sub-state machines
            //InitAmmoSearchHSM();
            // now put the machine into the actual initial state
            nextState = AmmoSearch;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            ;
        }
        break;

    case AmmoSearch: // in the first state, replace this with correct names
        // run sub-state machine for this state
        //NOTE: the SubState Machine runs and responds to events before anything in the this
        //state machine does
        //ThisEvent = RunAmmoSearchHSM(ThisEvent);
        switch (ThisEvent.EventType) {
            case TW_TRIGGERED:
                // check if rising edge
                if(ThisEvent.EventParam)
                {
                    nextState = AmmoLoad;
                    makeTransition = TRUE;
                }
                break;
            case ES_NO_EVENT:
            default:
                break;
        }
        break;
        
    case AmmoLoad:
        switch (ThisEvent.EventType) {
            case TW_TRIGGERED:
                // check if falling edge
                if(!ThisEvent.EventParam)
                {
                    nextState = AmmoSearch;
                    makeTransition = TRUE;
                }
                break;
            case BUMPED:
                if(ThisEvent.EventParam & PLUNGER_BUMPER)
                {
                    nextState = FirstTargetSearch;
                    makeTransition = TRUE;
                }
                break;
            case ES_NO_EVENT:
            default:
                break;
        }
        break;
        
    case FirstTargetSearch:
        switch (ThisEvent.EventType) {
                    //TS_FL_TRIGGERED for either go to firstargetunload
                    //TS_FR_TRIGGERED
            case TAPE_TRIGGERED:
                if(ThisEvent.EventParam && readBeaconDetector())
                {
                    nextState = FirstTargetUnload;
                    makeTransition = TRUE;
                }
                break;
            case ES_NO_EVENT:
            default:
                break;
        }
        break;
        
    case FirstTargetUnload:
        switch (ThisEvent.EventType) {
            case UNLOADED:
                nextState = SecondTargetSearch;
                makeTransition = TRUE;
                break;
            case ES_NO_EVENT:
            default:
                break;
        }
        break;
        
    case SecondTargetSearch:
        switch (ThisEvent.EventType) {
                    //TS_FL_TRIGGERED for either go to secondtargetunload
                    //TS_FR_TRIGGERED
            case TAPE_TRIGGERED:
                if(ThisEvent.EventParam && readBeaconDetector())
                {
                    nextState = FirstTargetUnload;
                    makeTransition = TRUE;
                }
                break;
            case ES_NO_EVENT:
            default:
                break;
        }
        break;
    
    case SecondTargetUnload:
        switch (ThisEvent.EventType) {
            case UNLOADED:
                nextState = AmmoSearch;
                makeTransition = TRUE;
                break;
            case ES_NO_EVENT:
            default:
                break;
        }
        break;
    
    default: // all unhandled states fall into here
        break;
    } // end switch on Current State

    if (makeTransition == TRUE) { // making a state transition, send EXIT and ENTRY
        // recursively call the current state with an exit event
        RunTopLevelHSM(EXIT_EVENT); // <- rename to your own Run function
        CurrentState = nextState;
        RunTopLevelHSM(ENTRY_EVENT); // <- rename to your own Run function
    }

    ES_Tail(); // trace call stack end
    return ThisEvent;
}