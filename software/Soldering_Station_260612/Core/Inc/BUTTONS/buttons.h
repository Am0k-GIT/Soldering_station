/***********************************************************************************************************

  Author:     Am0k
  Github:     https://github.com/Am0k-GIT

***********************************************************************************************************/

#ifndef BUTTON_H
#define BUTTON_H

#include "stm32f4xx_hal.h"

#define MAX_BUTTONS                    7                                       // Максимальное количество кнопок в системе
#define BUTTON_DEBOUNCE_TIME          20                                       // Время антидребезга (мс)
#define BUTTON_LONG_PRESS_TIME       700                                       // Время длинного нажатия (мс)

typedef enum
{
  BTN_NONE = 0, BTN_SHORT_PRESS, BTN_LONG_PRESS
} ButtonEvent_t;

typedef struct
{
  GPIO_TypeDef *GPIO_Port;
  uint16_t GPIO_Pin;
  uint32_t LastTick;
  uint8_t IsPressed;
  uint8_t StateChanged;
  ButtonEvent_t Event;
} Button_t;

void Button_Init (Button_t *btn, GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin);
void Button_Process (void);

#endif // BUTTON_H
