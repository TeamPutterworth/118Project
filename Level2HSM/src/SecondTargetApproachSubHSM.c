/*
 * File:   SecondTargetApproachHSM.c
 * Author: jcrowley
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
#include "SecondTargetApproachSubHSM.h"
#include "sensors.h"
#include "motor.h"
#include "IO_Ports.h"

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/

#define LEFT 1
#define RIGHT 0

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/

typedef enum {
    InitPState,
    Backward,
    TankTurn,
} HSMState_t;

static const char *StateNames[] = {
	"InitPState",
	"Backward",
	"TankTurn",
	"Forward",
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
 * @Function InitSecondTargetApproachSubHSM(uint8_t Priority)
 * @param Priority - internal variable to track which event queue to use
 * @return TRUE or FALSE
 * @brief This will get called by the framework at the beginning of the code
 *        execution. It will post an ES_INIT event to the appropriate event
 *        queue, which will be handled inside RunFirstTargetSearchSubHSM function.
 */
uint8_t InitSecondTargetApproachSubHSM(void)
{
    ES_Event returnEvent;

    CurrentState = InitPState;
    returnEvent = RunSecondTargetApproachSubHSM(INIT_EVENT);
    if (returnEvent.EventType == ES_NO_EVENT) {
        return TRUE;
    }
    return FALSE;
}

/**
 * @Function RunSecondTargetApproachSubHSM(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be responded.
 * @return Event - return event (type and param), in general should be ES_NO_EVENT
 * @brief This is the implementation for the top level of our state machine and 
 *        it contains the basic logic for our autonomous robot to meet the minimum
 *        specs of the competition.
 */
ES_Event RunSecondTargetApproachSubHSM(ES_Event ThisEvent)
{
    uint8_t makeTransition = FALSE; 
    HSMState_t nextState; 

    ES_Tattle(); // trace call stack

    switch (CurrentState) {
    case InitPState: // If current state is initial Pseudo State
        if (ThisEvent.EventType == ES_INIT)// only respond to ES_Init
        {
            nextState = Forward;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;


        }
        break;

    case Forward:
        switch(ThisEvent.EventType){
            case ES_ENTRY:
                moveForward();
                break;

            default:
                break;
        }
    case Backward:
        switch(ThisEvent.EventType){
            case ES_ENTRY:
                moveBackward()
                break;

            default:
                break;
        }
        break;

    case TankTurn:
        switch (ThisEvent.EventType) {  
            case ES_ENTRY:
                tankTurnRight();
                break;
            
            default:
                break;
        }
        break;
        
        break;

    default: // all unhandled states fall into here
        break;
    } // end switch on Current State

    if (makeTransition == TRUE) { // making a state transition, send EXIT and ENTRY
        // recursively call the current state with an exit event
        RunSecondTargetApproachSubHSM(EXIT_EVENT); 
        CurrentState = nextState;
        RunSecondTargetApproachSubHSM(ENTRY_EVENT); 

    }

    ES_Tail(); // trace call stack end
    return ThisEvent;
}