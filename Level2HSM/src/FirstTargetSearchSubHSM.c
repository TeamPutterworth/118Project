/*
 * File:   FirstTargetSearchSubHSM.h
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
#include "FirstTargetSearchSubHSM.h"
#include "sensors.h"
#include "motor.h"

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/

#define LEFT 0
#define RIGHT 1

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/

typedef enum {
    InitPState,
    ForwardScan,
    Scan,
    Forward,
    Align,
    Backward,
    TankTurn,
} HSMState_t;

static const char *StateNames[] = {
	"InitPState",
	"ForwardScan",
	"Scan",
	"Forward",
	"Align",
	"Backward",
	"TankTurn",
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

static uint8_t firstEntry = 1;
static uint8_t lastBump = LEFT;
/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

/**
 * @Function InitFirstTargetSearchSubHSM(uint8_t Priority)
 * @param Priority - internal variable to track which event queue to use
 * @return TRUE or FALSE
 * @brief This will get called by the framework at the beginning of the code
 *        execution. It will post an ES_INIT event to the appropriate event
 *        queue, which will be handled inside RunFirstTargetSearchSubHSM function.
 */
uint8_t InitFirstTargetSearchSubHSM(void)
{
    ES_Event returnEvent;

    CurrentState = InitPState;
    returnEvent = RunFirstTargetSearchSubHSM(INIT_EVENT);
    if (returnEvent.EventType == ES_NO_EVENT) {
        return TRUE;
    }
    return FALSE;
}

/**
 * @Function RunFirstTargetSearchSubHSM(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be responded.
 * @return Event - return event (type and param), in general should be ES_NO_EVENT
 * @brief This is the implementation for the top level of our state machine and 
 *        it contains the basic logic for our autonomous robot to meet the minimum
 *        specs of the competition.
 */
ES_Event RunFirstTargetSearchSubHSM(ES_Event ThisEvent)
{
    uint8_t makeTransition = FALSE; // use to flag transition
    HSMState_t nextState; // <- change type to correct enum
    static uint16_t scanTimer = TIMER_22_TICKS;
    static uint8_t turnParam;

    ES_Tattle(); // trace call stack

    switch (CurrentState) {
    case InitPState: // If current state is initial Pseudo State
        if (ThisEvent.EventType == ES_INIT)// only respond to ES_Init
        {
            nextState = ForwardScan;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            //ES_Timer_InitTimer(LONG_HSM_TIMER,3*LONG_TIMER_TICKS);
            
        }
        break;

    case ForwardScan:
        switch(ThisEvent.EventType) {
            case ES_ENTRY:
                if(turnParam == RIGHT)
                {
                    tankTurnRight();
                    ES_Timer_InitTimer(TIMER_22,scanTimer);
                    turnParam = LEFT;
                }
                else
                {    
                    tankTurnLeft();
                    ES_Timer_InitTimer(TIMER_22,scanTimer);
                    turnParam = RIGHT;
                }
                break;
            case BEACON_TRIGGERED:
                if(ThisEvent.EventParam)
                {
                    scanTimer = TIMER_22_TICKS;
                    nextState = Forward;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    ES_Timer_StopTimer(TIMER_22);
                }
                break;
            case ES_TIMEOUT:
                if(ThisEvent.EventParam == TIMER_22)
                {
                    scanTimer = scanTimer + TIMER_22_TICKS;
                    if(turnParam == RIGHT)
                    {
                        tankTurnRight();
                        ES_Timer_InitTimer(TIMER_22,scanTimer);
                        turnParam = LEFT;
                    }
                    else
                    {    
                        tankTurnLeft();
                        ES_Timer_InitTimer(TIMER_22,scanTimer);
                        turnParam = RIGHT;
                    }
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
           case BEACON_TRIGGERED:
                if(!ThisEvent.EventParam)
                {
                    nextState = ForwardScan;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                }
                break;
            case BUMPED:
                if(ThisEvent.EventParam & FL_BUMPER){
                    lastBump = LEFT;
                    nextState = Backward;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                }
                else if (ThisEvent.EventParam & FR_BUMPER)
                {
                    lastBump = RIGHT;
                    nextState = Backward;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;  
                }
                break;
            case ES_TIMEOUT:
                if(ThisEvent.EventParam & SCAN_TIMER)
                {
                    nextState = ForwardScan;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                }
                        
                break;
            case ES_EXIT:
                ES_Timer_StopTimer(LONG_HSM_TIMER);
                ES_Timer_StopTimer(MEDIUM_HSM_TIMER);
                ES_Timer_StopTimer(SCAN_TIMER);
                break;
            case ES_NO_EVENT:
            default:
                break;
        }
        break;
        
    case Backward:
        switch(ThisEvent.EventType){
            case ES_ENTRY:
                moveBackward();
                ES_Timer_InitTimer(MEDIUM_HSM_TIMER, MEDIUM_TIMER_TICKS);
                break;
            case ES_TIMEOUT:
                if(ThisEvent.EventParam == MEDIUM_HSM_TIMER)
                {
                    nextState = TankTurn;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    ES_Timer_InitTimer(TIMER_45, TIMER_45_TICKS);
                }
                break;
            case ES_EXIT:
                break;
        }
        break;
        
    case TankTurn:
        switch (ThisEvent.EventType) {  
            case ES_ENTRY:
                if(lastBump == RIGHT)
                {
                    tankTurnLeft();
                }
                else
                {
                    tankTurnRight();
                }
                break;
            case ES_TIMEOUT:
                if(ThisEvent.EventParam == TIMER_45){
                    nextState = Forward;
                    makeTransition  = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    ES_Timer_InitTimer(SCAN_TIMER, SCAN_TIMER_TICKS);
                }
                break;
            case ES_EXIT:
                break;
            default:
                break;
        }
        break;

        
    default: // all unhandled states fall into here
        break;
    } // end switch on Current State

    if (makeTransition == TRUE) { // making a state transition, send EXIT and ENTRY
        // recursively call the current state with an exit event
        RunFirstTargetSearchSubHSM(EXIT_EVENT); 
        CurrentState = nextState;
        RunFirstTargetSearchSubHSM(ENTRY_EVENT); 
    }

    ES_Tail(); // trace call stack end
    return ThisEvent;
}