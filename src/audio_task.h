/******************************************************************************
 * File Name: audio_task.h
 *
 * Description: This file is the public interface of audio_task.c source file
 *
 ********************************************************************************
 * Created by:  Bastiaan Slee
 *****************************************​**************************************/

/*******************************************************************************
 * Include guard
 ******************************************************************************/
#ifndef SOURCE_AUDIO_TASK_H_
#define SOURCE_AUDIO_TASK_H_

/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/*******************************************************************************
 * Global constants
 *******************************************************************************/

/* Define how many samples in a frame */
#define FRAME_SIZE                  4096u
/* Desired sample rate. Typical values: 8/16/22.05/32/44.1/48 kHz */
#define SAMPLE_RATE_HZ              44100u
/* Decimation Rate of the PDM/PCM block. Typical value is 64. DEC_RATE = 2 x SINC_RATE */
#define DECIMATION_RATE             64u
/* Audio Mode: LEFT / RIGHT / STEREO */
#define AUDIO_MODE					CYHAL_PDM_PCM_MODE_STEREO
/* Gain for Left and Right in DB */
#define GAIN_LEFT					CY_PDM_PCM_BYPASS
#define GAIN_RIGHT					CY_PDM_PCM_BYPASS
#define OCTAVES 					11

/* Audio Subsystem Clock. Typical values depends on the desire sample rate:
 - 8/16/48kHz    : 24.576 MHz
 - 22.05/44.1kHz : 22.574 MHz */
#define AUDIO_SYS_CLOCK_HZ          22574000u

/* PDM/PCM Pins */
#define PDM_DATA                    P10_5
#define PDM_CLK                     P10_4

/* Onboard LED */
#define PWM_LED_FREQ_HZ     		(1000000u)  /* in Hz */
/* subtracting from 100 since the LED is connected in active low configuration */
#define GET_DUTY_CYCLE(x)   		(100 - x)
/* Allowed TCPWM compare value for maximum brightness */
#define LED_MAX_BRIGHTNESS  		(100u)
/* Allowed TCPWM compare value for minimum brightness*/
#define LED_MIN_BRIGHTNESS  		(2u)

/* RTC: Structure tm stores years since 1900 */
#define TM_YEAR_BASE 				(1900u)
#define TM_MONTH_BASE 				(1u)

/* FFT Custom constants */
#define FFT_FORWARD 				0x01
#define FFT_REVERSE 				0x00

/* FFT Windowing type */
#define FFT_WIN_TYP_HANN 			0x02 /* hann */
#define FFT_WIN_TYP_FLT_TOP 		0x08 /* flat top */

/* FFT Mathematial constants */
#define twoPi 						6.28318531
#define fourPi 						12.56637061
#define sixPi 						18.84955593

/*******************************************************************************
 * Data structure and enumeration
 ******************************************************************************/

/*******************************************************************************
 * Global variable
 ******************************************************************************/

/* Z-weighting curves have no effect and are to represent the original readings
 * A-weighting curves from in steps of whole octaves
 * Weightings are standardised between 10Hz and 20kHz, which is based on what you can really hear.
 * https://en.wikipedia.org/wiki/A-weighting
 * https://www.nti-audio.com/en/support/know-how/frequency-weightings-for-sound-level-measurements
 *                             16Hz 31.5Hz   63Hz  125Hz 250Hz 500Hz 1kHz  2kHz  4kHz  8kHz 16kHz
 */
static float aWeighting[] = { -56.7, -39.4, -26.2, -16.1, -8.6, -3.2, 0.0,  1.2,  1.0, -1.1, -6.6 };
static float cWeighting[] = { -8.5,   -3.0,  -0.8,  -0.2,  0.0,  0.0, 0.0, -0.2, -0.8, -3.0, -8.5 };
static float zWeighting[] = {  0.0,    0.0,   0.0,   0.0,  0.0,  0.0, 0.0,  0.0,  0.0,  0.0,  0.0 };

/*******************************************************************************
 * Function prototype
 ******************************************************************************/
void audio_task(void* param);

#endif /* SOURCE_AUDIO_TASK_H_ */

/* [] END OF FILE  */
