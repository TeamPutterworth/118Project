/*
 * File:   AmmoLoadSubHSM.h
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
#include "AmmoLoadSubHSM.h"
#include "SyncSampling.h"
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
    TankTurn,
    Backward,
    Forward,
    PivotTurn,
    Shimmy,
} HSMState_t;

static const char *StateNames[] = {
	"InitPState",
	"TankTurn",
	"Backward",
	"Forward",
	"PivotTurn",
	"Shimmy",
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
static uint8_t shimmy = LEFT;
static uint8_t shimmyCount = 0;


/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

/**
 * @Function InitAmmoLoadSubHSM(uint8_t Priority)
 * @param Priority - internal variable to track which event queue to use
 * @return TRUE or FALSE
 * @brief This will get called by the framework at the beginning of the code
 *        execution. It will post an ES_INIT event to the appropriate event
 *        queue, which will be handled inside RunAmmoLoadSubHSM function.
 */
uint8_t InitAmmoLoadSubHSM(void)
{
    ES_Event returnEvent;
    
    shimmyCount = 0;
    
    CurrentState = InitPState;
    returnEvent = RunAmmoLoadSubHSM(INIT_EVENT);
    if (returnEvent.EventType == ES_NO_EVENT) {
        return TRUE;
    }
    return FALSE;
}

/**
 * @Function RunAmmoLoadSubHSM(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be responded.
 * @return Event - return event (type and param), in general should be ES_NO_EVENT
 * @brief This is the implementation for the top level of our state machine and 
 *        it contains the basic logic for our autonomous robot to meet the minimum
 *        specs of the competition.
 */
ES_Event RunAmmoLoadSubHSM(ES_Event ThisEvent)
{
    uint8_t makeTransition = FALSE; // use to flag transition
    HSMState_t nextState; // <- change type to correct enum
    uint8_t lastTape = getLastTape();
    static uint8_t pivotState = LEFT;
    ES_Tattle(); // trace call stack

    switch (CurrentState) {
    case InitPState: // If current state is initial Pseudo State
        if (ThisEvent.EventType == ES_INIT)// only respond to ES_Init
        {
            nextState = PivotTurn;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
        }
        break;
    case PivotTurn:
        switch (ThisEvent.EventType){
            case ES_ENTRY:
                if(lastTape){
                    pivotTurnLeftBackward();
                    pivotState = LEFT;
                }else{
                    pivotTurnRightBackward();
                    pivotState = RIGHT;
                }
                break;
            case TAPE_TRIGGERED:
                if(ThisEvent.EventParam & TS_BR && pivotState == RIGHT){
                    nextState = Forward;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                }else if(ThisEvent.EventParam & TS_BL && pivotState == LEFT){
                    nextState = Forward;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                }
                break;
        }
        break;
    case Forward:
        switch(ThisEvent.EventType)
        {
            case ES_ENTRY:
                moveForward();
                break;
            case TW_TRIGGERED:
                if(ThisEvent.EventParam & TW_B)
                {
                    nextState = TankTurn;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                }
                break;
        }
        break;
            
    case TankTurn:
        switch (ThisEvent.EventType) {  
            case ES_ENTRY:
                ES_Timer_InitTimer(TIMER_45, TIMER_45_TICKS);
                if(pivotState == RIGHT)
                {
                    tankTurnRight();
                }
                else if(pivotState == LEFT)
                {
                    tankTurnLeft();
                }
                break;
            case ES_TIMEOUT:
                if(ThisEvent.EventParam == TIMER_45){
                    nextState = Backward;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                }
                break;
            case ES_EXIT:
                ES_Timer_StopTimer(TIMER_45);
                break;
            case ES_NO_EVENT:
            default:
                break;
        }
        break;
        
    case Backward:
        switch (ThisEvent.EventType) {
            case ES_ENTRY:
                moveBackward();
                setMoveSpeed(40);
                ES_Timer_InitTimer(LONG_HSM_TIMER, LONG_TIMER_TICKS+500);
                break;
            case ES_TIMEOUT:
                if(ThisEvent.EventParam == LONG_HSM_TIMER)
                {
                    nextState = Shimmy;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                }
                break;
            case ES_EXIT:
                ES_Timer_StopTimer(LONG_HSM_TIMER);
                break;
            case ES_NO_EVENT:
            default:
                break;
        }
        break;
        
    case Shimmy:
        switch(ThisEvent.EventType){
            case ES_ENTRY:
                ES_Timer_InitTimer(SHIMMY_TIMER, SHIMMY_TIMER_TICKS/2);
                tankTurnLeft();
                break;
                
            case ES_TIMEOUT:
                if(ThisEvent.EventParam == SHIMMY_TIMER)
                {
                    shimmyCount++;
                    if(shimmy == LEFT)
                    {
                        shimmy = RIGHT;
                        tankTurnRight();
                    }
                    else if(shimmy == RIGHT)
                    {
                        shimmy = LEFT;
                        tankTurnLeft(); 
                    }
                     ES_Timer_InitTimer(SHIMMY_TIMER, SHIMMY_TIMER_TICKS);
                }
                if(shimmyCount == 6)
                {
                    ThisEvent.EventType = UNLOADED;
                }
                break;
        }
        break;
    default: // all unhandled states fall into here
        break;
    } // end switch on Current State

    if (makeTransition == TRUE) { // making a state transition, send EXIT and ENTRY
        // recursively call the current state with an exit event
        RunAmmoLoadSubHSM(EXIT_EVENT);
        CurrentState = nextState;
        RunAmmoLoadSubHSM(ENTRY_EVENT); 
    }

    ES_Tail(); // trace call stack end
    return ThisEvent;
}