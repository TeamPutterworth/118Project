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
#define STUCK 4
#define LEFT 0
#define RIGHT 1
/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/


typedef enum {
    InitPState,
    Forward,
    TankTurn,
    Backward,
    AlignToTape,
    FollowTape,
} HSMState_t;

static const char *StateNames[] = {
	"InitPState",
	"Forward",
	"TankTurn",
	"Backward",
	"AlignToTape",
	"FollowTape",
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
uint8_t InitAmmoSearchSubHSM(void)
{
    ES_Event returnEvent;

    CurrentState = InitPState;
    returnEvent = RunAmmoSearchSubHSM(INIT_EVENT);
    if (returnEvent.EventType == ES_NO_EVENT) {
        return TRUE;
    }
    return FALSE;
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
    static uint8_t turnParam = 0; // use this flag to turnCW or turnCCW
    static uint8_t stuckCounter; // use this to see if we are stuck!
    static uint8_t pivotFlag; // Gradual turns when pivotFLag is high have extra condition (this should really probably be sep state)
    
    HSMState_t nextState; // <- change type to correct enum

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

    case Forward: // in the first state, replace this with correct names
        // run sub-state machine for this state
        //NOTE: the SubState Machine runs and responds to events before anything in the this
        //state machine does
        //ThisEvent = RunAmmoSearchHSM(ThisEvent);
        switch (ThisEvent.EventType) {
            case ES_ENTRY:
                moveForward();
                break;
            // fl triggered then turn right, else if fr triggered turn left
            case TAPE_TRIGGERED:
                // If tape is triggered after hitting a track wire and we weren't following it, back up a lot
                if ((!getTrackWireVals()[0] || !getTrackWireVals()[1]) && getLastTape() == NOT_FOLLOWING){
                    break;
                }else if (ThisEvent.EventParam & TS_FR){
                    if (turnParam == RIGHT){
                        stuckCounter++;
                    }else{
                        stuckCounter = 0;
                    }
                    
                    if (stuckCounter > STUCK){
                        turnParam = RIGHT;
                    }else{
                        turnParam = LEFT;
                    }
                    nextState = AlignToTape;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                }else if (ThisEvent.EventParam & TS_FL){   
                    if (turnParam == LEFT){
                        stuckCounter++;
                    }else{
                        stuckCounter = 0;
                    }
                    
                    if (stuckCounter > STUCK){
                        turnParam = LEFT;
                    }else{
                        turnParam = RIGHT;
                    }
                    nextState = AlignToTape;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                }
                break;
            case BUMPED:
                // We kinda want to ignore back bumpers when going forward, who cares if a robot hit us
                if (ThisEvent.EventParam == FL_BUMPER){
                    if (turnParam == LEFT){
                        stuckCounter++;
                    }else{
                        stuckCounter = 0;
                    }
                    
                    if (stuckCounter > STUCK){
                        turnParam = LEFT;
                    }else{
                        turnParam = RIGHT;
                    }
                    nextState = Backward;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    ES_Timer_InitTimer(MEDIUM_HSM_TIMER, MEDIUM_TIMER_TICKS);
                }else if (ThisEvent.EventParam == FR_BUMPER){
                    if (turnParam == RIGHT){
                        stuckCounter++;
                    }else{
                        stuckCounter = 0;
                    }
                    
                    if (stuckCounter > STUCK){
                        turnParam = RIGHT;
                    }else{
                        turnParam = LEFT;
                    }
                    nextState = Backward;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    ES_Timer_InitTimer(MEDIUM_HSM_TIMER, MEDIUM_TIMER_TICKS);
                }
                break;
            case TW_TRIGGERED:
                // check if rising edge
                if((ThisEvent.EventParam & TW_F) && getLastTape() == NOT_FOLLOWING){
                    nextState = Backward;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    ES_Timer_InitTimer(LONG_HSM_TIMER, LONG_TIMER_TICKS);
                }
                break;
            case ES_NO_EVENT:
            default:
                break;
        }
        break;
        
    case TankTurn:
        switch (ThisEvent.EventType) {  
            case ES_ENTRY:
                if (turnParam == RIGHT){
                    tankTurnRight();
                }else{
                    tankTurnLeft();
                }
                break;
            case ES_TIMEOUT:
                if (ThisEvent.EventParam == TIMER_45 || ThisEvent.EventParam == TIMER_22){
                    nextState = Forward;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                }
                break;
            case ES_EXIT:
                ES_Timer_StopTimer(TIMER_22);
                ES_Timer_StopTimer(TIMER_45);
                break;
            case ES_NO_EVENT:
            default:
                break;
        }
        break;
        
    case AlignToTape:
        switch (ThisEvent.EventType) {  
            case ES_ENTRY:
                if (turnParam == RIGHT){
                    tankTurnRight();
                }else{
                    tankTurnLeft();
                }
                break;
            case TAPE_TRIGGERED:
                if(!(ThisEvent.EventParam & TS_FR) && (turnParam == LEFT) && !(ThisEvent.EventParam & TS_FL)){
                    turnParam = RIGHT;
                    gradualTurnRight(5);
                    makeTransition = TRUE;
                    nextState = FollowTape;
                    ThisEvent.EventType = ES_NO_EVENT;
                } else if(!(ThisEvent.EventParam & TS_FL) && (turnParam == RIGHT) && !(ThisEvent.EventParam & TS_FR)){
                    turnParam = LEFT;
                    gradualTurnLeft(5);
                    makeTransition = TRUE;
                    nextState = FollowTape;
                    ThisEvent.EventType = ES_NO_EVENT;
                }
                break;
            case ES_NO_EVENT:
            default:
                break;
        }
        break;
    case FollowTape:
        switch(ThisEvent.EventType){
            case TAPE_TRIGGERED:
                if(!(ThisEvent.EventParam & TS_FR) && (turnParam == LEFT)){
                    turnParam = RIGHT;
                    gradualTurnRight(10);
                } else if(!(ThisEvent.EventParam & TS_FL) && (turnParam == RIGHT)){
                    turnParam = LEFT;
                    gradualTurnLeft(10);
                }
                break;
            case BUMPED:
                // We kinda want to ignore back bumpers when going forward, who cares if a robot hit us
                if (ThisEvent.EventParam & FL_BUMPER){
                    if (turnParam == LEFT){
                        stuckCounter++;
                    }else{
                        stuckCounter = 0;
                    }
                    
                    if (stuckCounter > STUCK){
                        turnParam = LEFT;
                    }else{
                        turnParam = RIGHT;
                    }
                    nextState = Backward;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    ES_Timer_InitTimer(MEDIUM_HSM_TIMER, MEDIUM_TIMER_TICKS);
                }else if (ThisEvent.EventParam & FR_BUMPER){
                    if (turnParam == RIGHT){
                        stuckCounter++;
                    }else{
                        stuckCounter = 0;
                    }
                    
                    if (stuckCounter > STUCK){
                        turnParam = RIGHT;
                    }else{
                        turnParam = LEFT;
                    }
                    nextState = Backward;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    ES_Timer_InitTimer(MEDIUM_HSM_TIMER, MEDIUM_TIMER_TICKS);
                }
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
                break;
            case ES_TIMEOUT:
                if(ThisEvent.EventParam == MEDIUM_HSM_TIMER){
                    nextState = TankTurn;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    ES_Timer_InitTimer(TIMER_45, TIMER_45_TICKS);
                }
                if(ThisEvent.EventParam == LONG_HSM_TIMER){
                    nextState = TankTurn;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    ES_Timer_InitTimer(TIMER_22, TIMER_22_TICKS);
                }
                break;
            case ES_EXIT:
                ES_Timer_StopTimer(MEDIUM_HSM_TIMER);
                ES_Timer_StopTimer(LONG_HSM_TIMER);
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
        RunAmmoSearchSubHSM(EXIT_EVENT);
        CurrentState = nextState;
        RunAmmoSearchSubHSM(ENTRY_EVENT); 
    }

    ES_Tail(); // trace call stack end
    return ThisEvent;
}