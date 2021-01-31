/******************************************************************************
 * File Name: capsense_task.h
 *
 * Description: This file is the public interface of capsense_task.c source file.
 *
 ********************************************************************************
 * Created by:  Bastiaan Slee
 *****************************************​**************************************/

/*******************************************************************************
 *  Include guard
 ******************************************************************************/
#ifndef SOURCE_CAPSENSE_TASK_H_
#define SOURCE_CAPSENSE_TASK_H_

/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/*******************************************************************************
 * Global constants
 *******************************************************************************/
#define CAPSENSE_INTERRUPT_PRIORITY    (7u)
#define EZI2C_INTERRUPT_PRIORITY    (6u)    /* EZI2C interrupt priority must be
                                             * higher than CapSense interrupt */
#define CAPSENSE_SCAN_INTERVAL_MS    (10u)   /* in milliseconds*/

/*******************************************************************************
 * Data structure and enumeration
 ******************************************************************************/
typedef enum {
	CAPSENSE_SCAN, CAPSENSE_PROCESS
} capsense_command_t;

/*******************************************************************************
 * Global variable
 ******************************************************************************/
extern QueueHandle_t capsense_command_q;
extern uint32_t CapSense_slider_value;

/*******************************************************************************
 * Function prototype
 ******************************************************************************/
void capsense_task(void* param);

#endif /* SOURCE_CAPSENSE_TASK_H_ */

/* END OF FILE [] */
