/*
 * led_display.h
 *
 *  Created on: Aug 2, 2024
 *      Author: desd
 */

#ifndef INC_LED_DISPLAY_H_
#define INC_LED_DISPLAY_H_

void LED_Display(uint8_t LedStatus)
{
  /* Turn OFF all LEDs */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  switch(LedStatus)
  {
    case (1):
      /* Turn ON LED1 */
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
      break;

    case (2):
      /* Turn ON LED2 */
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
      break;

    case (3):
      /* Turn ON LED3 */
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
      break;

    case (4):
      /* Turn ON LED4 */
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
      break;
    default:
      break;
  }
}


#endif /* INC_LED_DISPLAY_H_ */
