/*
 * File:   AmmoSearchSubHSM.h
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
#include "AmmoSearchSubHSM.h"
#include "sensors.h"
#include "motor.h"

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/


typedef enum {
    InitPState,
    Forward,
    TankTurn,
    Backward,
} HSMState_t;

static const char *StateNames[] = {
	"InitPState",
	"Forward",
	"TankTurn",
	"Backward",
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

static HSMState_t CurrentState = InitPState; // <- change enum name to match ENUM
static uint8_t MyPriority;


/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

/**
 * @Function InitAmmoSearchSubHSM(uint8_t Priority)
 * @param Priority - internal variable to track which event queue to use
 * @return TRUE or FALSE
 * @brief This will get called by the framework at the beginning of the code
 *        execution. It will post an ES_INIT event to the appropriate event
 *        queue, which will be handled inside RunAmmoSearchSubHSM function.
 */
uint8_t InitAmmoSearchSubHSM(uint8_t Priority)
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
 * @Function PostAmmoSearchSubHSM(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be posted to queue
 * @return TRUE or FALSE
 * @brief This function is a wrapper to the queue posting function.
 */
uint8_t PostAmmoSearchSubHSM(ES_Event ThisEvent)
{
    return ES_PostToService(MyPriority, ThisEvent);
}

/**
 * @Function RunAmmoSearchSubHSM(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be responded.
 * @return Event - return event (type and param), in general should be ES_NO_EVENT
 * @brief This is the implementation for the top level of our state machine and 
 *        it contains the basic logic for our autonomous robot to meet the minimum
 *        specs of the competition.
 */
ES_Event RunAmmoSearchSubHSM(ES_Event ThisEvent)
{
    uint8_t makeTransition = FALSE; // use to flag transition
    static uint8_t turnParam; // use this flag to turnCW or turnCCW
    HSMState_t nextState; // <- change type to correct enum

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
            nextState = Forward;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            ;
        }
        break;

    case Forward: // in the first state, replace this with correct names
        // run sub-state machine for this state
        //NOTE: the SubState Machine runs and responds to events before anything in the this
        //state machine does
        //ThisEvent = RunAmmoSearchHSM(ThisEvent);
        switch (ThisEvent.EventType) {
            // fl triggered then turn right, else if fr triggered turn left
            case TAPE_TRIGGERED:
                if (ThisEvent.EventParam)
                {
                    turnParam = 1;
                    nextState = Backward;
                    makeTransition = TRUE;
                }
                break;
            case BUMPED:
                // We kinda want to ignore back bumpers when going forward, who cares if a robot hit us
                if (ThisEvent.EventParam == 0x1)
                {
                    turnParam = 0;
                    nextState = Backward;
                    makeTransition = TRUE;
                } 
                else if (ThisEvent.EventParam == 0x2)
                {
                    turnParam = 1;
                    nextState = Backward;
                    makeTransition = TRUE;
                }
                break;
            case ES_ENTRY:
                moveForward();
                break;
            case ES_NO_EVENT:
            default:
                break;
        }
        break;
        
    case TankTurn:
        switch (ThisEvent.EventType) {  
            case ES_ENTRY:
                ES_Timer_InitTimer(SHORT_HSM_TIMER, SHORT_TIMER_TICKS);
                if (turnParam)
                {
                    tankTurnRight();
                }
                else
                {
                    tankTurnLeft();
                }
                break;
            case ES_TIMEOUT:
                nextState = Forward;
                makeTransition = TRUE;
                break;
            case ES_NO_EVENT:
            default:
                break;
        }
        break;
        
    case Backward:
        switch (ThisEvent.EventType) {
            case ES_ENTRY:
                ES_Timer_InitTimer(SHORT_HSM_TIMER, SHORT_TIMER_TICKS);
                moveBackward();
                break;
            case ES_TIMEOUT:
                nextState = TankTurn;
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
        RunAmmoSearchSubHSM(EXIT_EVENT); // <- rename to your own Run function
        CurrentState = nextState;
        RunAmmoSearchSubHSM(ENTRY_EVENT); // <- rename to your own Run function
    }

    ES_Tail(); // trace call stack end
    return ThisEvent;
}