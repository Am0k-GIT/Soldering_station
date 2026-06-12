/***********************************************************************************************************
 *
 *  Created on: Mar 29, 2024
 *      Author: Am0k
 *
 *          Please, configurate your buttons manually or use CubeMX:
 *
 *          Declare your buttons in "main.h":
 *
 *      #define B0_Pin GPIO_PIN_12
 *      #define B0_GPIO_Port GPIOB
 *      #define B0_EXTI_IRQn EXTI15_10_IRQn
 *      #define B1_Pin GPIO_PIN_13
 *      #define B1_GPIO_Port GPIOB
 *      #define B1_EXTI_IRQn EXTI15_10_IRQn

 *          Configurate your buttons in "gpio.c":
 *
 *      GPIO_InitStruct.Pin = B0_Pin|B1_Pin;
 *      GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
 *      GPIO_InitStruct.Pull = GPIO_PULLUP;
 *      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
 *
 *      HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
 *      HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
 *
 *      You must include HAL driver for your MCU
 *      #include "stm32f1xx_hal.h"
 *
 *          You must define next variables in your "config.h" file
 *
 *      BUTTON_LATENCY_DEBOUNCE            - delay (mS), e.c. 10
 *      BUTTON_LATENCY_LONGPRESS           - delay (mS), e.c. 1000
 *
 *          Also you must enable interrupts that the buttons is connected to:
 *
 *      EXTI0_IRQn                        < EXTI Line0 Interrupt
 *      EXTI1_IRQn                        < EXTI Line1 Interrupt
 *      EXTI2_IRQn                        < EXTI Line2 Interrupt
 *      EXTI3_IRQn                        < EXTI Line3 Interrupt
 *      EXTI4_IRQn                        < EXTI Line4 Interrupt
 *      EXTI9_5_IRQn                      < External Line[9:5] Interrupts
 *      EXTI15_10_IRQn                    < External Line[15:10] Interrupts
 *
 *          Select PORTS of connected buttons in __GET_PORT[] array.
 *
 *          In your code you need to call the function
 *      BUTTON_IRQ_Debounce(void (*callback)(uint16_t key_number, bool longpress))
 *      what takes as an argument a click processing function.
 *
***********************************************************************************************************/

#ifndef BUTTONS_H
#define BUTTONS_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#define BUTTON_LATENCY_DEBOUNCE                  10                            // mS
#define BUTTON_LATENCY_LONGPRESS                 1000                          // mS

static GPIO_TypeDef* __GET_PORT[16] = {
		GPIOC,                   // pin_0
                0,                       // pin_1
                0,                       // pin_2
                0,                       // pin_3
                GPIOC,                   // pin_4
		GPIOC,                   // pin_5
                0,                       // pin_6
                0,                       // pin_7
                0,                       // pin_8
                0,                       // pin_9
                0,                       // pin_10
		GPIOC,                   // pin_11
                GPIOB,                   // pin_12
                GPIOB,                   // pin_13
		GPIOB,                   // pin_14
		GPIOB,                   // pin_15
};

static IRQn_Type __GET_IRQ[16] = {
                EXTI0_IRQn,              // pin_0
                EXTI1_IRQn,              // pin_1
                EXTI2_IRQn,              // pin_2
                EXTI3_IRQn,              // pin_3
                EXTI4_IRQn,              // pin_4
                EXTI9_5_IRQn,            // pin_5
                EXTI9_5_IRQn,            // pin_6
                EXTI9_5_IRQn,            // pin_7
                EXTI9_5_IRQn,            // pin_8
                EXTI9_5_IRQn,            // pin_9
                EXTI15_10_IRQn,          // pin_10
                EXTI15_10_IRQn,          // pin_11
                EXTI15_10_IRQn,          // pin_12
                EXTI15_10_IRQn,          // pin_13
                EXTI15_10_IRQn,          // pin_14
                EXTI15_10_IRQn,          // pin_15
};

uint8_t HAL_GPIO_GET_REAL_NUMBER (uint16_t pin);
extern void HAL_GPIO_EXTI_Callback (uint16_t);                                 // Callback внешнего прерывания
extern void BUTTON_IRQ_Debounce (void (*) (uint16_t, bool));                     // функция подавления дребезга

#endif /* INC_BUTTONS_BUTTONS__H_ */
