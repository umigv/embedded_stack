/*
 * ringled.h
 *
 *  Created on: Feb 10, 2023
 *      Author: Liam
 */

#ifndef INC_RINGLED_H_
#define INC_RINGLED_H_

#define NEOPIXEL_ZERO	34
#define NEOPIXEL_ONE	68
#define NUM_PIXELS	12
#define DMA_BUFF_SIZE	(NUM_PIXELS * 24) + 1

#include "stdbool.h"

bool is_blinking = false;

typedef union
{
  struct
  {
	uint8_t b;
	uint8_t r;
	uint8_t g;
  } color;
  uint32_t data;
} PixelRGB_t;

// Mystery function
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_3);
}

// Interrupt code
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// invert is_blinking boolean
    if (GPIO_Pin == GPIO_PIN_13) { // INT Source is pin A9
    	is_blinking = !is_blinking;
    }
}

// Handles blinking/solid color on led.
// Arguments:
// uint8_t RGB values for solid color: uint8_t r2, uint8_t g2, uint8_t b2,
// uint8_t RGB values for blinking color: uint8_t r, uint8_t g, uint8_t b,
// pointers to timer handle typedef 2 and 9: TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* htim9
void blink_or_not(uint8_t r2, uint8_t g2, uint8_t b2, uint8_t r, uint8_t g, uint8_t b, TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* htim9){

	// get an array of 'pixels'
	PixelRGB_t pixel[NUM_PIXELS] = {0};

	// mystery
	uint32_t dmaBuffer[DMA_BUFF_SIZE] = {0};
	uint32_t *pBuff;

	// get initial timer values
	uint16_t timer_val2 = __HAL_TIM_GET_COUNTER(htim9);
	uint16_t timer_val = __HAL_TIM_GET_COUNTER(htim9);

	// infinite loop
	while(1){

		// if blinking mode
		if (is_blinking){

			// if difference between current timer value and last timer value (time elapsed)
			// is greater than 1/4 of 8 bit timer (16384), then switch state
			if (__HAL_TIM_GET_COUNTER(htim9) - timer_val >= 16384){

				// off state
				if (pixel[0].color.g != 0 || pixel[0].color.b != 0 || pixel[0].color.r != 0){
					pixel[0].color.g = 0;
					pixel[0].color.r = 0;
					pixel[0].color.b = 0;
				}

				// on state
				else {
					pixel[0].color.g = g;
					pixel[0].color.r = r;
					pixel[0].color.b = b;

					// reduce brightness
					pixel[0].color.r >>= 2;
					pixel[0].color.g >>= 2;
					pixel[0].color.b >>= 2;
				}

				// set timer value for next iteration
				timer_val = __HAL_TIM_GET_COUNTER(htim9);
			}
		}

		// if solid color mode
		else {
			// set color
			pixel[0].color.g = g2;
			pixel[0].color.r = r2;
			pixel[0].color.b = b2;

			// reduce brightness
			pixel[0].color.r >>= 2;
			pixel[0].color.g >>= 2;
			pixel[0].color.b >>= 2;
		}

		// Sending the color to the neopixel:

		// sets a small buffer between setting colors (same timer process as before)
		// if there isn't a buffer it doesn't work for some reason
		if (__HAL_TIM_GET_COUNTER(htim9) - timer_val2 >= 256) {

			// copies pixel data to all pixels
			for (int i = 0; i < (NUM_PIXELS - 1); i++) {
				pixel[i+1].data = pixel[i].data;
			}

			// pixel buffer for every bit we need to send to neopixel (numpixels * 24)
			pBuff = dmaBuffer;

			// for each pixel, create the bits we need to send from pixel data
			for (int i = 0; i < NUM_PIXELS; i++) {
			   for (int j = 23; j >= 0; j--) {
				 if ((pixel[i].data >> j) & 0x01) {
				   *pBuff = NEOPIXEL_ONE;
				 }
				 else {
				   *pBuff = NEOPIXEL_ZERO;
				 }
				 pBuff++;
			   }
			}
			dmaBuffer[DMA_BUFF_SIZE - 1] = 0; // last element must be 0!

			// Starts the TIM PWM signal generation in DMA mode (from spec)
			HAL_TIM_PWM_Start_DMA(htim2, TIM_CHANNEL_3, dmaBuffer, DMA_BUFF_SIZE);

			// updates timer value for next iteration
			timer_val2 = __HAL_TIM_GET_COUNTER(htim9);
		}
	}
}

#endif /* INC_RINGLED_H_ */
