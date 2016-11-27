/*
 * File:   FirstTargetUnloadSubSubHSM.h
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
#include "FirstTargetUnloadSubHSM.h"
#include "sensors.h"
#include "motor.h"
#include "LED.h"

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/
#define RIGHT 1
#define LEFT 0
/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/

typedef enum {
    InitPState,
    PivotTurn,
    Forward,
    UnloadTwo,
    UnloadOne,     
} HSMState_t;

static const char *StateNames[] = {
	"InitPState",
	"PivotTurn",
	"Forward",
	"UnloadTwo",
	"UnloadOne",
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
static uint16_t servoPulse = UNLOADING_CENTER_PULSE;

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

/**
 * @Function InitFirstTargetUnloadSubHSM(uint8_t Priority)
 * @param Priority - internal variable to track which event queue to use
 * @return TRUE or FALSE
 * @brief This will get called by the framework at the beginning of the code
 *        execution. It will post an ES_INIT event to the appropriate event
 *        queue, which will be handled inside RunFirstTargetUnloadSubHSM function.
 */
uint8_t InitFirstTargetUnloadSubHSM(void)
{
    ES_Event returnEvent;

    CurrentState = InitPState;
    returnEvent = RunFirstTargetUnloadSubHSM(INIT_EVENT);
    if (returnEvent.EventType == ES_NO_EVENT) {
        return TRUE;
    }
    return FALSE;
}

/**
 * @Function RunFirstTargetUnloadSubHSM(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be responded.
 * @return Event - return event (type and param), in general should be ES_NO_EVENT
 * @brief This is the implementation for the top level of our state machine and 
 *        it contains the basic logic for our autonomous robot to meet the minimum
 *        specs of the competition.
 */
ES_Event RunFirstTargetUnloadSubHSM(ES_Event ThisEvent)
{
    static uint8_t turnParam;
    uint8_t makeTransition = FALSE; // use to flag transition
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
            //InitAmmoLoadSubHSM();
            // now put the machine into the actual initial state
            nextState = PivotTurn;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            ;
        }
        break;

    case PivotTurn:
        switch (ThisEvent.EventType) {  
            case ES_ENTRY:
                if (turnParam == RIGHT){
                    pivotTurnRight();
                } else if (turnParam == LEFT){
                    pivotTurnLeft();
                }
                break;
            case TAPE_TRIGGERED:
                if ((ThisEvent.EventParam & TS_FR) && (ThisEvent.EventParam & TS_FL))
                {
                    nextState = Forward;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                }
                else if (!((ThisEvent.EventParam & TS_FR) && (ThisEvent.EventParam & TS_FL)
                        && (ThisEvent.EventParam & TS_FM)))
                {
                    nextState = Forward;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                } else {
                    nextState = UnloadTwo;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                }
                break;
            case ES_NO_EVENT:
            default:
                break;
        }
        break;
        
    case Forward:
        switch (ThisEvent.EventType) {
            case ES_ENTRY:
                moveForward();
                break;
            case TAPE_TRIGGERED:
                if(!(ThisEvent.EventParam & TS_FR))
                {
                    turnParam = RIGHT;
                    nextState = PivotTurn;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                }
                else if(!(ThisEvent.EventParam & TS_FL))
                {
                    turnParam = LEFT;
                    nextState = PivotTurn;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                }
                else if(ThisEvent.EventParam & TS_FM)
                {
                    nextState = UnloadTwo;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                }
                break;
            case ES_NO_EVENT:
            default:
                break;
        }
        break;
    case UnloadTwo:
        switch (ThisEvent.EventType) {
            case ES_ENTRY:
                stopMoving();
                ES_Timer_InitTimer(SERVO_TIMER,SERVO_TIMER_TICKS);
                setPulseBridgeServo(BRIDGE_OUT_PULSE);
                setPulseUnloadingServo(servoPulse);
                break;
            case ES_TIMEOUT:
                if (ThisEvent.EventParam == SERVO_TIMER)
                {
                    ES_Timer_InitTimer(SERVO_TIMER,SERVO_TIMER_TICKS);
                    if (servoPulse < UNLOADING_HIGH_PULSE)
                    {
                        servoPulse+=10;
                        setPulseUnloadingServo(servoPulse);
                    } else {
                        nextState = UnloadOne;
                        makeTransition = TRUE;
                    }
                    ThisEvent.EventType = ES_NO_EVENT;
                }
                break;
            case ES_NO_EVENT:
            default:
                break;
        }
        break;
    case UnloadOne:
        switch (ThisEvent.EventType) {
            case ES_ENTRY:
                stopMoving();
                ES_Timer_InitTimer(SERVO_TIMER,SERVO_TIMER_TICKS);
                setPulseUnloadingServo(servoPulse);
                break;
            case ES_TIMEOUT:
                if (ThisEvent.EventParam == SERVO_TIMER)
                {
                    if (servoPulse > UNLOADING_LOW_PULSE)
                    {
                        ES_Timer_InitTimer(SERVO_TIMER,SERVO_TIMER_TICKS);
                        servoPulse-=10;
                        setPulseUnloadingServo(servoPulse);
                        ThisEvent.EventType = ES_NO_EVENT;
                    } else {
                        servoPulse = UNLOADING_CENTER_PULSE;
                        setPulseUnloadingServo(servoPulse);
                        ES_Timer_InitTimer(LONG_HSM_TIMER,LONG_TIMER_TICKS);
                    }
                } else if(ThisEvent.EventParam == LONG_HSM_TIMER){
                    ThisEvent.EventType = UNLOADED;
                    setPulseBridgeServo(BRIDGE_IN_PULSE);
                }
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
        RunFirstTargetUnloadSubHSM(EXIT_EVENT); // <- rename to your own Run function
        CurrentState = nextState;
        RunFirstTargetUnloadSubHSM(ENTRY_EVENT); // <- rename to your own Run function
    }

    ES_Tail(); // trace call stack end
    return ThisEvent;
}