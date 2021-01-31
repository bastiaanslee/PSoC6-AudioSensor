/******************************************************************************
 * File Name: main.c
 *
 * Description: This code does record audio samples with the PDM microphone,
 * 			   converts that to PCM data. Which is then going through a
 * 			   Fast Fourier Transform with HANN windowing, to be able to split
 * 			   the audio data into octaves. Based on A/C/Z weighting, the
 * 			   audio data is then further calculated into data that represents
 * 			   what our ears do actually hear.
 *
 * 			   Read documentation, find all code and discuss optimisations for this project:
 * 			   https://www.electromaker.io/project/view/PSoC6-AudioSensor
 *
 *              This code uses FreeRTOS. For documentation and API references
 *              of FreeRTOS, visit : https://www.freertos.org
 *
 *******************************************************************************
 * Created by:  Bastiaan Slee
 *******************************************************************************
 *
 * Credits go to all these pages that did help me getting to understand parts of it, summing
 * together to this project. I've read so many pages, that I'm I did forgot some of them to list here.
 *
 *
 * Information about Sound, Audio and processing it:
 *   Wikipedia:
 *     Your ears						https://en.wikipedia.org/wiki/Ear
 *     Fourier Transform				https://en.wikipedia.org/wiki/Fourier_transform
 *     Joseph Fourier				https://en.wikipedia.org/wiki/Joseph_Fourier
 *     Window functions				https://en.wikipedia.org/wiki/Window_function
 *     Hann Window					https://en.wikipedia.org/wiki/Hann_function
 *   NIH: Journey of Sound to the Brain
 *   								https://upload.wikimedia.org/wikipedia/commons/7/72/Journey_of_Sound_to_the_Brain.ogv
 *   NTI-audio: Frequency-Weightings for Sound Level Measurements
 *   								https://www.nti-audio.com/en/support/know-how/frequency-weightings-for-sound-level-measurements
 *   3Blue1Brown: What is the Fourier Transform? A visual introduction
 *   								https://youtu.be/spUNpyF58BY
 *   Steve L. Brunton: Fourier Analysis series
 *   								https://www.youtube.com/playlist?list=PLMrJAkhIeNNT_Xh3Oy0Y4LTj0Oxo8GqsC
 *   Elan Ness-Cohn: Developing An Intuition for Fourier Transforms
 *   								https://sites.northwestern.edu/elannesscohn/2019/07/30/developing-an-intuition-for-fourier-transforms/
 *   Stack Overflow					https://stackoverflow.com/a/4678313
 * 		  							https://stackoverflow.com/a/604756
 *
 * Technical information and code examples:
 *   FreeRTOS						https://www.freertos.org/
 *     Tasks:						https://www.freertos.org/taskandcr.html
 *     Queues:						https://www.freertos.org/Embedded-RTOS-Queues.html
 *   Cypress documentation:
 *     WiFi Connection Manager Library:
 *     								https://cypresssemiconductorco.github.io/wifi-connection-manager/api_reference_manual/html/index.html
 *     Secure Sockets: 				https://cypresssemiconductorco.github.io/secure-sockets/api_reference_manual/html/index.html
 *     Hardware Abstraction Layer (for PDM/PCM, RTC):
 *     								https://cypresssemiconductorco.github.io/psoc6hal/html/index.html
 *   Ed Boel: Noice Level Meter		https://bitbucket.org/edboel/edboel/src/master/noise/src/
 *   TTNApeldoorn: LoRaSoundkit		https://github.com/TTNApeldoorn/sound-sensor
 *   Enrique Condes: arduinoFFT		https://github.com/kosme/arduinoFFT
 *   David Lettier: NTP client:		https://lettier.github.io/posts/2016-04-26-lets-make-a-ntp-client-in-c.html
 *   Nicolas Seriot: NTP datagram:	http://seriot.ch/ntp.php#21
 *   Wikipedia: ASCI escape codes:	https://en.wikipedia.org/wiki/ANSI_escape_code
 *   InfluxDB: Write data with the InfluxDB API:
 *   								https://docs.influxdata.com/influxdb/v2.0/write-data/developer-tools/api/
 *
 *****************************************​**************************************/

/******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cy_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* My 4 tasks */
#include "wifi_task.h"
#include "influx_task.h"
#include "capsense_task.h"
#include "audio_task.h"

/*******************************************************************************
 * Global constants
 ******************************************************************************/
/* Priorities of user tasks in this project. configMAX_PRIORITIES is defined in
 * the FreeRTOSConfig.h and higher priority numbers denote high priority tasks.
 */
#define TASK_CAPSENSE_PRIORITY      (1)
#define TASK_AUDIO_PRIORITY         (2)
#define TASK_INFLUX_PRIORITY    	(3)
#define TASK_WIFI_PRIORITY   		(4)

/* Stack sizes of user tasks in this project: https://www.freertos.org/FAQMem.html#StackSize */
#define TASK_CAPSENSE_STACK_SIZE    (5 * 1024)
#define TASK_AUDIO_STACK_SIZE       (5 * 1024)
#define TASK_WIFI_STACK_SIZE 		(5 * 1024)
#define TASK_INFLUX_STACK_SIZE  	(5 * 1024)

/* Queue lengths of message queues used in this project */
#define SINGLE_ELEMENT_QUEUE        (1u)

/*******************************************************************************
 * Global Variables
 ********************************************************************************/
/* This enables RTOS aware debugging. */
volatile int uxTopUsedPriority;

/*******************************************************************************
 * Function Name: main()
 ********************************************************************************
 * Summary:
 *  System entrance point. This function sets up user tasks and then starts
 *  the RTOS scheduler.
 *
 * Return:
 *  int
 *
 *******************************************************************************/
int main(void) {
	cy_rslt_t result;

	/* For debugging the WiFi connection, I had to enable this function. */
	//cy_log_init(CY_LOG_DEBUG4, NULL, NULL);

	/* This enables RTOS aware debugging in OpenOCD. */
	uxTopUsedPriority = configMAX_PRIORITIES - 1;

	/* Initialize the device and board peripherals */
	result = cybsp_init();
	if (result != CY_RSLT_SUCCESS) {
		/* Board init failed. Stop program execution */
		CY_ASSERT(0);
	}

	/* Enable global interrupts */
	__enable_irq();

	/* Initialize retarget-io to use the debug UART port. */
	result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
	CY_RETARGET_IO_BAUDRATE);
	CY_ASSERT(result == CY_RSLT_SUCCESS);

	/* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen.
	 * For more coding and used colours, see https://en.wikipedia.org/wiki/ANSI_escape_code
	 */
	printf("\033[m"); // reset colour to standard
	printf("\x1b[2J\x1b[;H");
	printf("\033[100m"); // background colour to "Bright Black (Gray)"
	printf("==================================================================\r\n");
	printf("===                                                            ===\r\n");
	printf("===    \033[93mBastiaan Slee - PSoC6 Audio Sensor             \033[97m         ===\r\n");
	printf("===                                                            ===\r\n");
	printf("==================================================================\r\n");
	printf("\033[m"); // reset colour to standard
	printf("\r\n");

	/* Create the queues. See the respective data-types for details of queue
	 * contents. There is one used for the CapSense interrupt changes,
	 * and one for the communication between the Audio task and Influx task
	 */
	capsense_command_q = xQueueCreate(SINGLE_ELEMENT_QUEUE,
			sizeof(capsense_command_t));
	influx_command_data_q = xQueueCreate(200, sizeof(influx_command_data_t));

	/* Create the user tasks. See the respective task definition for more
	 * details of these tasks.
	 */
	xTaskCreate(capsense_task, "CapSense Task", TASK_CAPSENSE_STACK_SIZE, NULL,
			TASK_CAPSENSE_PRIORITY, NULL);
	xTaskCreate(audio_task, "Audio Task", TASK_AUDIO_STACK_SIZE, NULL,
			TASK_AUDIO_PRIORITY, NULL);
	xTaskCreate(wifi_task, "WIFI Task", TASK_WIFI_STACK_SIZE, NULL,
			TASK_WIFI_PRIORITY, NULL);
	xTaskCreate(influx_task, "Influx Task", TASK_INFLUX_STACK_SIZE, NULL,
			TASK_INFLUX_PRIORITY, NULL);

	/* Start the RTOS scheduler. This function should never return */
	vTaskStartScheduler();

	/*~~~~~~~~~~~~~~~~~~~~~ Should never get here! ~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	/* RTOS scheduler exited */
	/* Halt the CPU if scheduler exits */
	CY_ASSERT(0);
}

/* [] END OF FILE  */
