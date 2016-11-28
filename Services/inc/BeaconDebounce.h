// Joseph Grant
// Midterm Problem 8
// 11/1/16

#ifndef BEACON_DEBOUNCE_H  // <- This should be changed to your own guard on both
#define BEACON_DEBOUNCE_H  //    of these lines


/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/

#include "ES_Configure.h"   // defines ES_Event, INIT_EVENT, ENTRY_EVENT, and EXIT_EVENT

/*******************************************************************************
 * PUBLIC #DEFINES                                                             *
 ******************************************************************************/


/*******************************************************************************
 * PUBLIC TYPEDEFS                                                             *
 ******************************************************************************/


/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/
/**
 * @Function uint8_t getBeaconVal()
 * @param Priority - internal variable to track which event queue to use
 * @return uint8_t
 * @brief This function returns private variable module for current status of the 
 * beacon.
 */
uint8_t getBeaconVal();

/**
 * @Function InitBeaconDebounceService(uint8_t Priority)
 * @param Priority - internal variable to track which event queue to use
 * @return TRUE or FALSE
 * @brief This will get called by the framework at the beginning of the code
 *        execution. It will post an ES_INIT event to the appropriate event
 *        queue, which will be handled inside RunTemplateService function. Remember
 *        to rename this to something appropriate.
 *        Returns TRUE if successful, FALSE otherwise
 */
uint8_t InitBeaconDebounceService(uint8_t Priority);

/**
 * @Function PostBeaconDebounceService(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be posted to queue
 * @return TRUE or FALSE
 * @brief This function is a wrapper to the queue posting function, and its name
 *        will be used inside ES_Configure to point to which queue events should
 *        be posted to. Remember to rename to something appropriate.
 *        Returns TRUE if successful, FALSE otherwise
 */
uint8_t PostBeaconDebounceService(ES_Event ThisEvent);

/**
 * @Function RunBeaconDebounceService(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be responded.
 * @return Event - return event (type and param), in general should be ES_NO_EVENT
 * @brief This function is where you implement the whole of the service,
 *        as this is called any time a new event is passed to the event queue. 
 * @note Remember to rename to something appropriate.
 *       Returns ES_NO_EVENT if the event have been "consumed." 
 */
ES_Event RunBeaconDebounceService(ES_Event ThisEvent);



#endif /* BEACON_DEBOUNCE_H */

