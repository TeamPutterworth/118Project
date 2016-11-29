/*
 * File:   SecondTargetSearchSubHSM.h
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
#include "SecondTargetSearchSubHSM.h"
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
    GradualTurn,
    Scan,
} HSMState_t;

static const char *StateNames[] = {
	"InitPState",
	"Backward",
	"TankTurn",
	"GradualTurn",
	"Scan",
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

static uint8_t direction = LEFT;
static uint8_t difference = 21;


/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

/**
 * @Function InitSecondTargetSearchSubHSM(uint8_t Priority)
 * @param Priority - internal variable to track which event queue to use
 * @return TRUE or FALSE
 * @brief This will get called by the framework at the beginning of the code
 *        execution. It will post an ES_INIT event to the appropriate event
 *        queue, which will be handled inside RunFirstTargetSearchSubHSM function.
 */
uint8_t InitSecondTargetSearchSubHSM(void)
{
    ES_Event returnEvent;

    CurrentState = InitPState;
    returnEvent = RunSecondTargetSearchSubHSM(INIT_EVENT);
    if (returnEvent.EventType == ES_NO_EVENT) {
        return TRUE;
    }
    return FALSE;
}

/**
 * @Function RunSecondTargetSearchSubHSM(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be responded.
 * @return Event - return event (type and param), in general should be ES_NO_EVENT
 * @brief This is the implementation for the top level of our state machine and 
 *        it contains the basic logic for our autonomous robot to meet the minimum
 *        specs of the competition.
 */
ES_Event RunSecondTargetSearchSubHSM(ES_Event ThisEvent)
{
    uint8_t makeTransition = FALSE; 
    HSMState_t nextState; 

    ES_Tattle(); // trace call stack

    switch (CurrentState) {
    case InitPState: // If current state is initial Pseudo State
        if (ThisEvent.EventType == ES_INIT)// only respond to ES_Init
        {
            nextState = GradualTurn;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
        }
        break;

    case Backward:
        switch(ThisEvent.EventType){
            case ES_ENTRY:
                moveBackward();
                break;
            case ES_TIMEOUT:
                if(ThisEvent.EventParam == MEDIUM_HSM_TIMER)
                {
                    nextState = TankTurn;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    ES_Timer_InitTimer(TIMER_180, TIMER_180_TICKS);
                }
                break;
            case ES_EXIT:
                ES_Timer_StopTimer(LONG_HSM_TIMER);
                ES_Timer_StopTimer(MEDIUM_HSM_TIMER);
                break;
        }
        break;

    case TankTurn:
        switch (ThisEvent.EventType) {  
            case ES_ENTRY:
                if(direction == RIGHT)
                {
                    tankTurnRight();
                }
                else
                {
                    tankTurnLeft();
                }
                break;
            case ES_TIMEOUT:
                if(ThisEvent.EventParam & (TIMER_90 | TIMER_180)){
                    nextState = GradualTurn;
                    makeTransition  = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    ES_Timer_InitTimer(TIMER_45, TIMER_45_TICKS);
                }
                break;
            case ES_EXIT:
                ES_Timer_StopTimer(TIMER_90);
                ES_Timer_StopTimer(TIMER_180);
            
            default:
                break;
        }
        break;

    case GradualTurn:
        switch(ThisEvent.EventType){
            case ES_ENTRY:

                // circle the last target and increase radius each time
                if(direction == LEFT)
                {
                    gradualTurnLeft(difference);
                    direction = RIGHT;
                }
                else
                {
                    gradualTurnRight(difference);
                    direction = LEFT;
                }
                difference -= 3;
  
                break;
            case TAPE_TRIGGERED:
                if(ThisEvent.EventParam & (TS_FL | TS_FR | TS_FM))
                {
                    nextState = Backward;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    ES_Timer_InitTimer(MEDIUM_HSM_TIMER, MEDIUM_TIMER_TICKS);
                }
                break;
            case BUMPED:
                if(ThisEvent.EventParam & (FL_BUMPER | FR_BUMPER))
                {
                    nextState = Backward;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    ES_Timer_InitTimer(MEDIUM_HSM_TIMER, MEDIUM_TIMER_TICKS);
                }
                break;
            /*case ES_TIMEOUT:
                if(ThisEvent.EventParam == TIMER_45)
                {
                    nextState = Scan; 
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    ES_Timer_InitTimer(TIMER_45, TIMER_45_TICKS);     
                }
                break;*/
                        
        }
        break;
    /*case Scan:
        switch(ThisEvent.EventType){
            case ES_ENTRY:
                if(direction == LEFT)
                {
                    tankTurnLeft();
                    direction = RIGHT;
                }
                else
                {
                    tankTurnRight();
                    direction = LEFT;
                }
                break;
                
            case ES_TIMEOUT:
                if(ThisEvent.EventParam == TIMER_45)
                {
                    if(direction == RIGHT)
                    {
                        tankTurnLeft();
                    }
                    else
                    {
                        tankTurnRight();
                    }
                    ES_Timer_InitTimer(TIMER_22, TIMER_22_TICKS);
                }
                else if(ThisEvent.EventParam == TIMER_22)
                {
                    nextState = GradualTurn;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                }
                break;
        }
        
        
        break;*/
            
    default: 
        break;
    } 

    if (makeTransition == TRUE) { 
        // recursively call the current state with an exit event
        RunSecondTargetSearchSubHSM(EXIT_EVENT); 
        CurrentState = nextState;
        RunSecondTargetSearchSubHSM(ENTRY_EVENT); 
    }

    ES_Tail(); // trace call stack end
    return ThisEvent;
}