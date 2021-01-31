/******************************************************************************
 * File Name: audio_task.c
 *
 * Description: This file contains the task that handles PDM microphone audio.
 * 			   which is calculated into separate octaves.
 *
 ********************************************************************************
 * Created by:  Bastiaan Slee
 *****************************************​**************************************/

/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"
#include "cy_retarget_io.h"
#include "stdlib.h"
#include "stddef.h"
#include "time.h"
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* My related tasks */
#include "audio_task.h"
#include "influx_task.h"
#include "capsense_task.h"

/*******************************************************************************
 * Global functions
 ******************************************************************************/
void pdm_pcm_isr_handler(void *arg, cyhal_pdm_pcm_event_t event);
void clock_init(void);

void soundsensor_integerToFloat(int16_t *samples, float *vReal, float *vImag,
		uint16_t size);
void soundsensor_calculateEnergy(float *vReal, float *vImag, uint16_t samples);
void soundsensor_sumEnergy(const float *vReal, float *vEnergy, uint16_t size,
		uint16_t octaves);
void soundsensor_loudnessWeighting(float *vEnergy, float *weighting,
		float *spectrum, uint16_t octaves);
void soundsensor_toDecibel(float *spectrum, uint16_t octaves);

void arduinoFFT_Windowing(uint8_t windowType, uint8_t dir, float *vReal,
		uint16_t size);
void arduinoFFT_Compute(uint8_t dir, float *vReal, float *vImag, uint16_t size);
uint8_t arduinoFFT_Exponent(uint16_t value);
void arduinoFFT_Swap(float *x, float *y);
float sq(float num);

/*******************************************************************************
 * Global variable
 ******************************************************************************/
volatile bool pdm_pcm_flag = false;

/* HAL Object */
cyhal_pdm_pcm_t pdm_pcm;
cyhal_clock_t audio_clock;
cyhal_clock_t pll_clock;

/* HAL Config */
const cyhal_pdm_pcm_cfg_t pdm_pcm_cfg = {
		.sample_rate = SAMPLE_RATE_HZ,
		.decimation_rate = DECIMATION_RATE,
		.mode = AUDIO_MODE,
		.word_length = 16, /* bits */
		.left_gain = GAIN_LEFT, /* dB */
		.right_gain = GAIN_RIGHT, /* dB */
};

/* Received and calculated audio data */
int16_t audio_frame[FRAME_SIZE] = { 0 };
float audio_real[FRAME_SIZE];
float audio_imag[FRAME_SIZE];
float audio_energy[OCTAVES];

float audio_z_spectrum[OCTAVES + 1] = { 0.0 };
float audio_a_spectrum[OCTAVES + 1] = { 0.0 };

float audio_z_avg[OCTAVES + 1] = { 0.0 };
float audio_a_avg[OCTAVES + 1] = { 0.0 };

/* Onboard LED */
cyhal_pwm_t pwm_led;

/* Volume variables */
uint32_t corrected_volume = 0;
uint32_t noise_threshold = 1;

/* RTC object */
cyhal_rtc_t rtc_obj;
struct tm date_time; /* To hold the current RTC time */
struct tm avg_date_time; /* No it is not the AVG of the date/time ;) */
int avg_count = 0;

/*******************************************************************************
 * Function Name: audio_task
 ********************************************************************************
 * Summary:
 *  Task that controls the PDM microphone capture, processes the data, output
 *  to COM port, LED and Influx task.
 *
 * Parameters:
 *  void *param : Task parameter defined during task creation (unused)
 *
 *******************************************************************************/
void audio_task(void* param) {
	cy_rslt_t result;

	/* Immediately suspend the task to give WIFI connection priority */
	vTaskSuspend( NULL);
	printf("\033[94mAudio: task started!\033[m\n");

	/* Variables used for storing command and data for LED Task */
	influx_command_data_t influx_cmd_data;

	/* Init the clocks */
	clock_init();

	/* Initialize the PDM/PCM block */
	cyhal_pdm_pcm_init(&pdm_pcm, PDM_DATA, PDM_CLK, &audio_clock, &pdm_pcm_cfg);
	cyhal_pdm_pcm_register_callback(&pdm_pcm, pdm_pcm_isr_handler, NULL);
	cyhal_pdm_pcm_enable_event(&pdm_pcm, CYHAL_PDM_PCM_ASYNC_COMPLETE,
			CYHAL_ISR_PRIORITY_DEFAULT, true);
	cyhal_pdm_pcm_start(&pdm_pcm);

	/* Initialize RTC */
	result = cyhal_rtc_init(&rtc_obj);
	if (CY_RSLT_SUCCESS != result) {
		printf("\033[91mAudio: Failed to init RTC.\033[m\n");
		CY_ASSERT(0);
	}

	/* Read the time stored in the RTC */
	result = cyhal_rtc_read(&rtc_obj, &date_time);
	if (CY_RSLT_SUCCESS != result) {
		printf("\033[91mRTC: Failed to read.\033[m\n");
	}
	printf("\033[94mTimer: current time is %s\033[m", asctime(&date_time));
	cyhal_rtc_read(&rtc_obj, &avg_date_time);

	/* Initialize a PWM resource for driving an LED. */
	cyhal_pwm_init(&pwm_led, CYBSP_USER_LED, NULL);
	cyhal_pwm_set_duty_cycle(&pwm_led, GET_DUTY_CYCLE(LED_MAX_BRIGHTNESS),
			PWM_LED_FREQ_HZ);
	cyhal_pwm_start(&pwm_led);

	/* Setup to read the next frame */
	cyhal_pdm_pcm_read_async(&pdm_pcm, audio_frame, FRAME_SIZE);

	printf("   31.5Hz     63Hz    125Hz    250Hz    500Hz     1kHz     2kHz     4kHz     8kHz    16kHz    32kHz       Total\n");

	/* Repeatedly running part of the task */
	for (;;) {

		if (pdm_pcm_flag) {
			/* Clear the PDM/PCM flag */
			pdm_pcm_flag = false;

			/* Read current time from the RTC to test of reachable*/
			cyhal_rtc_read(&rtc_obj, &date_time);

			if ((long) mktime(&date_time) > (long) mktime(&avg_date_time)) {
				/* We are in the next second, calculate averages of the previous second! */
				for (int i = 0; i < OCTAVES; i++) {
					audio_z_avg[i] /= (float) avg_count;
					audio_a_avg[i] /= (float) avg_count;
				}

				/* calculate the dB of the averages */
				soundsensor_toDecibel(audio_z_avg, OCTAVES);
				soundsensor_toDecibel(audio_a_avg, OCTAVES);

				/* Send off the Z weighted readings to the Influx task */
				influx_cmd_data.influx_command = INFLUX_DATA;
				influx_cmd_data.audio_weighting = "z";
				influx_cmd_data.audio_octave01 = audio_z_avg[0];
				influx_cmd_data.audio_octave02 = audio_z_avg[1];
				influx_cmd_data.audio_octave03 = audio_z_avg[2];
				influx_cmd_data.audio_octave04 = audio_z_avg[3];
				influx_cmd_data.audio_octave05 = audio_z_avg[4];
				influx_cmd_data.audio_octave06 = audio_z_avg[5];
				influx_cmd_data.audio_octave07 = audio_z_avg[6];
				influx_cmd_data.audio_octave08 = audio_z_avg[7];
				influx_cmd_data.audio_octave09 = audio_z_avg[8];
				influx_cmd_data.audio_octave10 = audio_z_avg[9];
				influx_cmd_data.audio_octave11 = audio_z_avg[10];
				influx_cmd_data.audio_volume = audio_z_avg[OCTAVES];
				influx_cmd_data.sec_since_epoch = (long) mktime(&avg_date_time);
				xQueueSendToBack(influx_command_data_q, &influx_cmd_data, 0u);

				/* Send off the A weighted readings to the Influx task */
				influx_cmd_data.influx_command = INFLUX_DATA;
				influx_cmd_data.audio_weighting = "a";
				influx_cmd_data.audio_octave01 = audio_a_avg[0];
				influx_cmd_data.audio_octave02 = audio_a_avg[1];
				influx_cmd_data.audio_octave03 = audio_a_avg[2];
				influx_cmd_data.audio_octave04 = audio_a_avg[3];
				influx_cmd_data.audio_octave05 = audio_a_avg[4];
				influx_cmd_data.audio_octave06 = audio_a_avg[5];
				influx_cmd_data.audio_octave07 = audio_a_avg[6];
				influx_cmd_data.audio_octave08 = audio_a_avg[7];
				influx_cmd_data.audio_octave09 = audio_a_avg[8];
				influx_cmd_data.audio_octave10 = audio_a_avg[9];
				influx_cmd_data.audio_octave11 = audio_a_avg[10];
				influx_cmd_data.audio_volume = audio_a_avg[OCTAVES];
				influx_cmd_data.sec_since_epoch = (long) mktime(&avg_date_time);
				xQueueSendToBack(influx_command_data_q, &influx_cmd_data, 0u);

				/* reset the spectrum output */
				avg_count = 0;
				avg_date_time = date_time;
				for (int i = 0; i < OCTAVES; i++) {
					audio_z_avg[i] = 0.0;
					audio_a_avg[i] = 0.0;
				}
			}
			/* Last second average completed, now process new audio data */

			/* Convert the 16 bit integer to a float */
			soundsensor_integerToFloat(audio_frame, audio_real, audio_imag,
					FRAME_SIZE);

			/* apply HANN window, optimal for energy calculations */
			arduinoFFT_Windowing(FFT_WIN_TYP_HANN, FFT_FORWARD, audio_real,
					FRAME_SIZE);

			/* do FFT processing */
			arduinoFFT_Compute(FFT_FORWARD, audio_real, audio_imag, FRAME_SIZE);

			/* calculate energy */
			soundsensor_calculateEnergy(audio_real, audio_imag, FRAME_SIZE);

			/* sum up energy in bin for each octave */
			soundsensor_sumEnergy(audio_real, audio_energy, FRAME_SIZE,
					OCTAVES);

			/* update the spectrum with new measurements */
			soundsensor_loudnessWeighting(audio_energy, zWeighting,
					audio_z_spectrum, OCTAVES);
			soundsensor_loudnessWeighting(audio_energy, aWeighting,
					audio_a_spectrum, OCTAVES);

			/* Add the reading to the AVG */
			avg_count += 1;
			for (int i = 0; i < OCTAVES; i++) {
				audio_z_avg[i] += audio_z_spectrum[i];
				audio_a_avg[i] += audio_a_spectrum[i];
			}

			/* calculate the dB */
			soundsensor_toDecibel(audio_z_spectrum, OCTAVES);
			soundsensor_toDecibel(audio_a_spectrum, OCTAVES);

			/* Print the spectrum dB values from the A-Weighting.
			 * Skip the 16Hz bin as we cannot hear that anyway */
			for (int i = 1; i < OCTAVES; i++) {
				printf(" %8.1f", audio_a_spectrum[i]);
			}

			/* Print the total dB value */
			printf("    %8.1f ", audio_a_spectrum[OCTAVES]);

			/* Create a bar to show the total dB values, divided by 10 to keep it readable.
			 * Range is up to 150, which means you will be deaf by that time...
			 */
			printf("[\033[93m"); /* foreground color to "Bright Yellow" */
			for (int index = 0; index < 15; index++) {
				printf("%s",
						(index < floor(audio_a_spectrum[OCTAVES] / 10) ?
								"=" : " "));
			}
			printf("\033[m]"); /* reset background color to standard */
			printf("\n");

			/* Calculate the brightness of the LED */
			corrected_volume =
					floor(audio_a_spectrum[OCTAVES]) > CapSense_slider_value ?
							(uint32_t) floor(audio_a_spectrum[OCTAVES])
									- CapSense_slider_value :
							0;

			if (corrected_volume <= 0) {
				/* Stop PWM to turn the LED off */
				cyhal_pwm_stop(&pwm_led);
			} else {
				/* make sure the LED brightness is between the MIN and MAX */
				corrected_volume =
						corrected_volume < LED_MIN_BRIGHTNESS ?
								LED_MIN_BRIGHTNESS : corrected_volume;
				corrected_volume =
						corrected_volume > LED_MAX_BRIGHTNESS ?
								LED_MAX_BRIGHTNESS : corrected_volume;

				/* Start PWM to turn the LED on and drive the LED with brightness */
				cyhal_pwm_set_duty_cycle(&pwm_led,
						GET_DUTY_CYCLE(corrected_volume), PWM_LED_FREQ_HZ);
				cyhal_pwm_start(&pwm_led);
			}

			/* Read the next audio frame */
			cyhal_pdm_pcm_read_async(&pdm_pcm, audio_frame, FRAME_SIZE);
		}

		/* delay for a very short time, to let the other tasks with a lower prio go ahead */
		vTaskDelay(pdMS_TO_TICKS(1u));
	}
}

/*******************************************************************************
 * Function Name: pdm_pcm_isr_handler
 ********************************************************************************
 * Summary:
 *  PDM/PCM ISR handler. Set a flag to be processed in the main loop.
 *
 * Parameters:
 *  arg: not used
 *  event: event that occurred, not used
 *
 *******************************************************************************/
void pdm_pcm_isr_handler(void *arg, cyhal_pdm_pcm_event_t event) {
	(void) arg;
	(void) event;

	pdm_pcm_flag = true;
}

/*******************************************************************************
 * Function Name: clock_init
 ********************************************************************************
 * Summary:
 *  Initialize the clocks in the system.
 *
 *******************************************************************************/
void clock_init(void) {
	/* Initialize the PLL */
	cyhal_clock_get(&pll_clock, &CYHAL_CLOCK_PLL[0]);
	cyhal_clock_init(&pll_clock);
	cyhal_clock_set_frequency(&pll_clock, AUDIO_SYS_CLOCK_HZ, NULL);
	cyhal_clock_set_enabled(&pll_clock, true, true);

	/* Initialize the audio subsystem clock (CLK_HF[1])
	 * The CLK_HF[1] is the root clock for the I2S and PDM/PCM blocks */
	cyhal_clock_get(&audio_clock, &CYHAL_CLOCK_HF[1]);
	cyhal_clock_init(&audio_clock);

	/* Source the audio subsystem clock from PLL */
	cyhal_clock_set_source(&audio_clock, &pll_clock);
	cyhal_clock_set_enabled(&audio_clock, true, true);
}

/*******************************************************************************
 * Function Name: soundsensor_integerToFloat
 ********************************************************************************
 * Summary:
 *  Convert 16 bits from integer to float
 *  But it turns out, our PDM/PCM doesn't store 16 bits. It only stores 8 bits. (even if we did set it up that way...)
 *  And it doesn't put these bits in the high bytes as others do, but at the low bytes. Which makes it easier!
 *
 *  Uncomment the three lines if you want to have the incoming data visible for debugging.
 *
 *  Used these 2 project as an inspiration and source:
 *   https://bitbucket.org/edboel/edboel/src/master/noise/
 *   https://github.com/TTNApeldoorn/sound-sensor
 *
 *******************************************************************************/
void soundsensor_integerToFloat(int16_t *samples, float *vReal, float *vImag,
		uint16_t size) {
	int bit_size = 8;
	for (uint16_t i = 0; i < size; i++) {
		// char buffer [bit_size+1];
		// itoa(samples[i],buffer,2);
		// printf("%ld        %032s         %f\n",samples[i], buffer, (float)((samples[i] / pow(2,bit_size-1)));

		vReal[i] = (float) ((samples[i] / pow(2, bit_size - 1)));
		vImag[i] = 0.0;
	}
}

/*******************************************************************************
 * Function Name: arduinoFFT_Windowing
 ********************************************************************************
 * Summary:
 *  Weighting factors are computed once before multiple use of FFT
 *  The weighing function is symmetric; half the weighs are recorded
 *
 *  Used this project as a source:
 *   https://github.com/kosme/arduinoFFT
 *
 *******************************************************************************/
void arduinoFFT_Windowing(uint8_t windowType, uint8_t dir, float *vReal,
		uint16_t size) {
	double samplesMinusOne = (double) size - 1.0;
	for (uint16_t i = 0; i < (size >> 1); i++) {
		double indexMinusOne = (double) i;
		double ratio = (indexMinusOne / samplesMinusOne);
		double weightingFactor = 1.0;
		/* Compute and record weighting factor for windowing */
		switch (windowType) {
		case FFT_WIN_TYP_HANN: {
			weightingFactor = 0.54 * (1.0 - cos(twoPi * ratio));
			break;
		}
		default: /* Invalid command */
		{
			/* You could handle an invalid command here */
			break;
		}
		}
		if (dir == FFT_FORWARD) {
			vReal[i] *= weightingFactor;
			vReal[size - (i + 1)] *= weightingFactor;
		} else {
			vReal[i] /= weightingFactor;
			vReal[size - (i + 1)] /= weightingFactor;
		}
	}
}

/*******************************************************************************
 * Function Name: arduinoFFT_Compute
 ********************************************************************************
 * Summary:
 *  Computes in-place complex-to-complex FFT.
 *
 *  Used this project as a source:
 *   https://github.com/kosme/arduinoFFT
 *
 *******************************************************************************/
void arduinoFFT_Compute(uint8_t dir, float *vReal, float *vImag, uint16_t size) {
	/* Reverse bits */
	uint16_t j = 0;
	for (uint16_t i = 0; i < (size - 1); i++) {
		if (i < j) {
			arduinoFFT_Swap(&vReal[i], &vReal[j]);
			if (dir == FFT_REVERSE)
				arduinoFFT_Swap(&vImag[i], &vImag[j]);
		}
		uint16_t k = (size >> 1);
		while (k <= j) {
			j -= k;
			k >>= 1;
		}
		j += k;
	}
	/* Compute the FFT  */
	double c1 = -1.0;
	double c2 = 0.0;
	uint16_t l2 = 1;
	for (uint8_t l = 0; (l < arduinoFFT_Exponent(size)); l++) {
		uint16_t l1 = l2;
		l2 <<= 1;
		double u1 = 1.0;
		double u2 = 0.0;
		for (j = 0; j < l1; j++) {
			for (uint16_t i = j; i < size; i += l2) {
				uint16_t i1 = i + l1;
				double t1 = u1 * vReal[i1] - u2 * vImag[i1];
				double t2 = u1 * vImag[i1] + u2 * vReal[i1];
				vReal[i1] = vReal[i] - t1;
				vImag[i1] = vImag[i] - t2;
				vReal[i] += t1;
				vImag[i] += t2;
			}
			double z = ((u1 * c1) - (u2 * c2));
			u2 = ((u1 * c2) + (u2 * c1));
			u1 = z;
		}
		c2 = sqrt((1.0 - c1) / 2.0);
		c1 = sqrt((1.0 + c1) / 2.0);
		if (dir == FFT_FORWARD) {
			c2 = -c2;
		}
	}
	/* Scaling for reverse transform */
	if (dir != FFT_FORWARD) {
		for (uint16_t i = 0; i < size; i++) {
			vReal[i] /= size;
			vImag[i] /= size;
		}
	}
}

/*******************************************************************************
 * Function Name: soundsensor_calculateEnergy
 ********************************************************************************
 * Summary:
 *  Calculates energy from Real and Imaginary parts and place it back in the
 *  Real part. Only for half of the dataset!
 *
 *  Used these 2 projects as an inspiration and a source:
 *   https://bitbucket.org/edboel/edboel/src/master/noise/
 *   https://github.com/TTNApeldoorn/sound-sensor
 *
 *******************************************************************************/
void soundsensor_calculateEnergy(float *vReal, float *vImag, uint16_t size) {
	for (uint16_t i = 0; i < size / 2; i++) {
		vReal[i] = sqrt(sq(vReal[i]) + sq(vImag[i]));
	}
}

/*******************************************************************************
 * Function Name: soundsensor_sumEnergy
 ********************************************************************************
 * Summary:
 *  Sums up energy in whole octave bins.
 *
 *  We are working here with 44100 samples/sec with 4096 points, we can sample
 *  10.7666 Hz per step, for 2048 steps, so we only use half of the dataset!
 *  That still is a total of 22 kHz, which is above our hearing range.
 *
 *  NOTE: octave bins are square blocks in this code, with the determined Hz value
 *  being in the middle of the block. but in real life they have curves and overlap.
 *  That isn't implemented here! That is a TODO, but would require to swap the
 *  sumEnergy and loudnessWeighting steps.
 *
 *  Used these 2 projects as an inspiration and a source:
 *   https://bitbucket.org/edboel/edboel/src/master/noise/
 *   https://github.com/TTNApeldoorn/sound-sensor
 *
 *******************************************************************************/
void soundsensor_sumEnergy(const float *vReal, float *vEnergy, uint16_t size,
		uint16_t octaves) {
	/* Only skip the first bin (0 - 10.7 Hz), and start counting with the second bin
	 * This way we also collect 16Hz bin data.
	 * Although not hearable, it is nice for the Z-weighted figures */
	int bin_size = 1;

	int bin = bin_size;
	for (int octave = 0; octave < octaves; octave++) {
		float sum = 0.0;
		for (int i = 0; i < bin_size && bin < size / 2; i++) {
			sum += vReal[bin++];
		}
		/* The energy is a sum per full octave, and then either summed or averaged to your liking.
		 * Remember octave bins are logarithmic, so the higher bins, have more data points */
		vEnergy[octave] = (sum / bin_size); /* The average energy per full octave.  For the higher octaves, the average is smoothed out better */
		// vEnergy[octave] = sum;  /* The total energy in the full octave. For the higher octaves, there are more data points, and thus a relative higher energy. */
		bin_size *= 2;
		// printf("sumEnergy octave=%d, bin=%d, sum=%f\n", octave, bin-1, sum);
	}
}

/*******************************************************************************
 * Function Name: soundsensor_loudnessWeighting
 ********************************************************************************
 * Summary:
 *  Calculates the loudness input, according to A/C/Z-weighting scale.
 *
 *  Negative weightings don't make sense. So in that case we assume there
 *  is no sound on this level. Set everything below 1 to 1.
 *
 *  Used these 2 projects as an inspiration and a source:
 *   https://bitbucket.org/edboel/edboel/src/master/noise/
 *   https://github.com/TTNApeldoorn/sound-sensor
 *
 *******************************************************************************/
void soundsensor_loudnessWeighting(float *vEnergy, float *weighting,
		float *spectrum, uint16_t octaves) {
	float calc_energy = 0.0;
	for (int i = 0; i < octaves; i++) {
		calc_energy = vEnergy[i] * sqrt(pow(10, weighting[i] / 10.0));
		spectrum[i] = (calc_energy > 1 ? calc_energy : 1);
	}
}

/*******************************************************************************
 * Function Name: soundsensor_toDecibel
 ********************************************************************************
 * Summary:
 *  Convert energy to to dB
 *  If in soundsensor_calculateEnergy() we didn't SQRT vReal it would be 10*log10()
 *  But we did SQRT the value there, so we need to use 20*log10() here
 *
 *  Used these 2 projects as an inspiration and a source:
 *   https://bitbucket.org/edboel/edboel/src/master/noise/
 *   https://github.com/TTNApeldoorn/sound-sensor
 *
 *******************************************************************************/
void soundsensor_toDecibel(float *spectrum, uint16_t octaves) {
	float full_spectrum = 0.0;

	/* calculate for each band the dB */
	for (int i = 0; i < octaves; i++) {
		full_spectrum += spectrum[i];
		spectrum[i] = 20.0 * log10(spectrum[i]);
	}
	/* this is the total over all octaves */
	spectrum[octaves] = 20.0 * log10(full_spectrum);
}

/*******************************************************************************
 * Function Name: arduinoFFT_Exponent
 ********************************************************************************
 * Summary:
 *  Calculates the base 2 logarithm of a value.
 *
 *  Used this project as a source:
 *   https://github.com/kosme/arduinoFFT
 *
 *******************************************************************************/
uint8_t arduinoFFT_Exponent(uint16_t value) {
	uint8_t result = 0;
	while (((value >> result) & 1) != 1)
		result++;
	return (result);
}

/*******************************************************************************
 * Function Name: arduinoFFT_Swap
 ********************************************************************************
 * Summary:
 *  Swaps two float values.
 *
 *  Used this project as a source:
 *   https://github.com/kosme/arduinoFFT
 *
 *******************************************************************************/
void arduinoFFT_Swap(float *x, float *y) {
	float temp = *x;
	*x = *y;
	*y = temp;
}

/*******************************************************************************
 * Function Name: sq
 ********************************************************************************
 * Summary:
 *  C language doesn't have a SQ() defined where C++ has one.
 *  This is the simples solution for code compatibility: just create one!
 *  Alternative would be using:  pow(num, 2)
 *
 *******************************************************************************/
float sq(float num) {
	return (num * num);
}

/* END OF FILE [] */
