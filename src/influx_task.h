/******************************************************************************
 * File Name:   influx_task.h
 *
 * Description: This file is the public interface of influx_task.c source file.
 *
 *******************************************************************************
 * Created by:  Bastiaan Slee
 *****************************************​**************************************/

/*******************************************************************************
 *  Include guard
 ******************************************************************************/
#ifndef INFLUX_TASK_H_
#define INFLUX_TASK_H_

/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/*******************************************************************************
 * Global constants
 *******************************************************************************/
#define MAKE_IPV4_ADDRESS(a, b, c, d)     ((((uint32_t) d) << 24) | \
                                          (((uint32_t) c) << 16) | \
                                          (((uint32_t) b) << 8) |\
                                          ((uint32_t) a))

/* Change the server IP address to match the TCP server address (IP address of the PC). */
#define TCP_SERVER_IP_ADDRESS             MAKE_IPV4_ADDRESS(192, 168, 1, 245)
#define TCP_SERVER_PORT                   9999

/* Buffer size to store the incoming messages from the server. */
#define MAX_TCP_RECV_BUFFER_SIZE          (5 * 1024)
/* Maximum number of connection retries to the TCP server. */
#define MAX_TCP_SERVER_CONN_RETRIES       (3u)

/* InfluxDB setup */
#define INFLUX_ORGANISATION					"YOUR_ORGANISATION"
#define INFLUX_BUCKET						"AudioSensor"
#define INFLUX_TOKEN						"YOUR_TOKEN"
#define INFLUX_PRECISSION					"s" /* ns - Nanoseconds  us - Microseconds  ms - Milliseconds  s - Seconds */

/*******************************************************************************
 * Data structure and enumeration
 ******************************************************************************/
/* Available audio commands */
typedef enum {
	INFLUX_DATA,
} influx_command_t;

/* Structure used for storing audio data */
typedef struct {
	influx_command_t influx_command;

	char* audio_weighting;
	float audio_octave01;
	float audio_octave02;
	float audio_octave03;
	float audio_octave04;
	float audio_octave05;
	float audio_octave06;
	float audio_octave07;
	float audio_octave08;
	float audio_octave09;
	float audio_octave10;
	float audio_octave11;
	float audio_volume;

	long sec_since_epoch;
} influx_command_data_t;

/*******************************************************************************
 * Global variable
 ******************************************************************************/
extern QueueHandle_t influx_command_data_q;

/*******************************************************************************
 * Function Prototype
 ********************************************************************************/
void influx_task(void *arg);

#endif /* INFLUX_TASK_H_ */

/* [] END OF FILE */
