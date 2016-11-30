/*
 * File:   SecondTargetUnloadHSM.c
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
#include "SecondTargetUnloadSubHSM.h"
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
    AlignToTape,
    PivotTurn,
    Forward,
    UnloadOne,
    UnloadTwo,
    Backward,
    TankTurn,
} HSMState_t;

static const char *StateNames[] = {
	"InitPState",
	"AlignToTape",
	"PivotTurn",
	"Forward",
	"UnloadOne",
	"UnloadTwo",
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

static uint16_t servoPulse = UNLOADING_CENTER_PULSE;
static HSMState_t CurrentState = InitPState; // <- change enum name to match ENUM
static uint8_t MyPriority;

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

/**
 * @Function InitSecondTargetUnloadSubHSM(uint8_t Priority)
 * @param Priority - internal variable to track which event queue to use
 * @return TRUE or FALSE
 * @brief This will get called by the framework at the beginning of the code
 *        execution. It will post an ES_INIT event to the appropriate event
 *        queue, which will be handled inside RunFirstTargetSearchSubHSM function.
 */
uint8_t InitSecondTargetUnloadSubHSM(void)
{
    ES_Event returnEvent;

    CurrentState = InitPState;
    returnEvent = RunSecondTargetUnloadSubHSM(INIT_EVENT);
    if (returnEvent.EventType == ES_NO_EVENT) {
        return TRUE;
    }
    return FALSE;
}

/**
 * @Function RunSecondTargetUnloadSubHSM(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be responded.
 * @return Event - return event (type and param), in general should be ES_NO_EVENT
 * @brief This is the implementation for the top level of our state machine and 
 *        it contains the basic logic for our autonomous robot to meet the minimum
 *        specs of the competition.
 */
ES_Event RunSecondTargetUnloadSubHSM(ES_Event ThisEvent)
{
    static uint8_t turnParam;
    uint8_t makeTransition = FALSE; 
    HSMState_t nextState; 

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
            nextState = Backward;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            
        }
        break;

    case AlignToTape:
        switch (ThisEvent.EventType) {  
            case ES_ENTRY:
                if (turnParam == RIGHT){
                    tankTurnRight();
                } else if (turnParam == LEFT){
                    tankTurnLeft();
                }
                break;
            case TAPE_TRIGGERED:
                if(ThisEvent.EventParam & TS_FM)
                {
                    nextState = UnloadTwo;
                    makeTransition = TRUE;
                    ThisEvent.EventParam = ES_NO_EVENT;
                }
                else if((ThisEvent.EventParam & TS_FL) &&  (turnParam == RIGHT))
                {
                    moveForward();
                    setMoveSpeed(10);
                }
                else if((ThisEvent.EventParam & TS_FR) &&  (turnParam == LEFT))
                {
                    moveForward();
                    setMoveSpeed(10);
                }
                break;
            case ES_NO_EVENT:
            default:
                break;
        }
        break;
    case PivotTurn:
        switch (ThisEvent.EventType) {  
            case ES_ENTRY:
                if (turnParam == RIGHT){
                    pivotTurnLeft();
                } else if (turnParam == LEFT){
                    pivotTurnRight();
                }
                break;
            case TAPE_TRIGGERED:
                if((ThisEvent.EventParam & TS_FL) &&  (turnParam == RIGHT))
                {
                    nextState = Forward;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                }
                else if((ThisEvent.EventParam & TS_FR) &&  (turnParam == LEFT))
                {
                    nextState = Forward;
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
                
                if(ThisEvent.EventParam & TS_FM){
                    nextState = UnloadTwo;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                } 
                else if(ThisEvent.EventParam & TS_FL)
                {
                    turnParam = LEFT;
                    nextState = AlignToTape;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                }
                else if(ThisEvent.EventParam & TS_FR)
                {
                    turnParam = RIGHT;
                    nextState = AlignToTape;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
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
                ES_Timer_InitTimer(LONG_HSM_TIMER,MEDIUM_TIMER_TICKS);
                break;
            case TAPE_TRIGGERED:
//                if(ThisEvent.EventParam & TS_BR || ThisEvent.EventParam & TS_BL)
//                {
//                    nextState = TankTurn;
//                    makeTransition = TRUE;
//                    ThisEvent.EventType = ES_NO_EVENT;  
//                }
                break;
            case ES_TIMEOUT:
                if(ThisEvent.EventParam == MEDIUM_HSM_TIMER)
                {
                    nextState = TankTurn;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                }
                else if(ThisEvent.EventParam == LONG_HSM_TIMER)
                {
                    nextState = Forward;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                }
                break;
        }
        break;
    case TankTurn:
        switch (ThisEvent.EventType) {
            case ES_ENTRY:
                tankTurnLeft();
                ES_Timer_InitTimer(TIMER_90,TIMER_90_TICKS);
                break;
            case ES_TIMEOUT:
                if(ThisEvent.EventParam == TIMER_90)
                {
                    ThisEvent.EventType = UNLOADED;
                }
               
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
                    if (servoPulse < UNLOADING_HIGH_PULSE)
                    {
                        ES_Timer_InitTimer(SERVO_TIMER,SERVO_TIMER_TICKS);
                        servoPulse+=10;
                        setPulseUnloadingServo(servoPulse);
                    } else {
                        ES_Timer_InitTimer(LONG_HSM_TIMER,2.5*LONG_TIMER_TICKS);
                    }
                    
                }
                else if(ThisEvent.EventParam == LONG_HSM_TIMER)
                {
                    servoPulse = UNLOADING_CENTER_PULSE;
                    setPulseUnloadingServo(servoPulse);
                    nextState = UnloadOne;
                    makeTransition = TRUE;
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
                        ES_Timer_InitTimer(LONG_HSM_TIMER,2.5*LONG_TIMER_TICKS);
                    }
                }
                else if(ThisEvent.EventParam == LONG_HSM_TIMER)
                {
                    servoPulse = UNLOADING_CENTER_PULSE;
                    setPulseUnloadingServo(servoPulse);
                    nextState = Backward;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    ES_Timer_InitTimer(MEDIUM_HSM_TIMER,MEDIUM_TIMER_TICKS);
                }
//                case ES_TIMEOUT:
//                if (ThisEvent.EventParam == SERVO_TIMER)
//                {
//                    if (servoPulse < UNLOADING_HIGH_PULSE)
//                    {
//                        ES_Timer_InitTimer(SERVO_TIMER,SERVO_TIMER_TICKS);
//                        servoPulse+=10;
//                        setPulseUnloadingServo(servoPulse);
//                    } else {
//                        //nextState = Backward;
//                        //makeTransition = TRUE;
//                        ES_Timer_InitTimer(LONG_HSM_TIMER,2.5*LONG_TIMER_TICKS);
//                    }
//                    
//                }
//                else if(ThisEvent.EventParam == LONG_HSM_TIMER)
//                {
//                    nextState = Backward;
//                    makeTransition = TRUE;
//                    ThisEvent.EventType = ES_NO_EVENT;
//                    ES_Timer_InitTimer(MEDIUM_HSM_TIMER,MEDIUM_TIMER_TICKS);
//                }
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
        RunSecondTargetUnloadSubHSM(EXIT_EVENT); 
        CurrentState = nextState;
        RunSecondTargetUnloadSubHSM(ENTRY_EVENT); 
    }

    ES_Tail(); // trace call stack end
    return ThisEvent;
}