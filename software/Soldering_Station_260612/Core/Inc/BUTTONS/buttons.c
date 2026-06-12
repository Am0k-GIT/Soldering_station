/***********************************************************************************************************

  Author:     Am0k
  Github:     https://github.com/Am0k-GIT

***********************************************************************************************************/

#include "BUTTONS/buttons.h"

static Button_t *button_list[MAX_BUTTONS];
static uint8_t button_count = 0;

static IRQn_Type Get_Button_IRQn (uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case GPIO_PIN_0:
      return EXTI0_IRQn;
    case GPIO_PIN_1:
      return EXTI1_IRQn;
    case GPIO_PIN_2:
      return EXTI2_IRQn;
    case GPIO_PIN_3:
      return EXTI3_IRQn;
    case GPIO_PIN_4:
      return EXTI4_IRQn;
    case GPIO_PIN_5:
    case GPIO_PIN_6:
    case GPIO_PIN_7:
    case GPIO_PIN_8:
    case GPIO_PIN_9:
      return EXTI9_5_IRQn;
    default:
      return EXTI15_10_IRQn;
  }
}

void Button_Init (Button_t *btn, GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin)
{
  btn->GPIO_Port = GPIO_Port;
  btn->GPIO_Pin = GPIO_Pin;
  btn->LastTick = 0;
  btn->IsPressed = 0;
  btn->StateChanged = 0;
  btn->Event = BTN_NONE;

  if (button_count < MAX_BUTTONS)
  {
    button_list[button_count] = btn;
    button_count++;
  }
}

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
  IRQn_Type irq_line = Get_Button_IRQn (GPIO_Pin);
  HAL_NVIC_DisableIRQ (irq_line);
  for (uint8_t i = 0; i < button_count; i++)
  {
    if (button_list[i]->GPIO_Pin == GPIO_Pin)
    {
      button_list[i]->LastTick = HAL_GetTick ();                               // Фиксируем время прерывания
      button_list[i]->StateChanged = 1;                                        // Взводим флаг для обработки в таймере
      break;
    }
  }
  HAL_NVIC_EnableIRQ (irq_line);
}

void Button_Process (void)
{
  uint32_t current_time = HAL_GetTick ();

  for (uint8_t i = 0; i < button_count; i++)
  {
    Button_t *btn = button_list[i];

    // Чтение физического состояния (1 - нажата, 0 - отпущена)
    uint8_t current_pin_state = (HAL_GPIO_ReadPin (btn->GPIO_Port, btn->GPIO_Pin) == GPIO_PIN_RESET);

    // 1. Обработка стабильного состояния ПОСЛЕ завершения дребезга
    if (btn->StateChanged)
    {
      if ((current_time - btn->LastTick) >= BUTTON_DEBOUNCE_TIME)
      {
        btn->StateChanged = 0; // Дребезг завершен, состояние стабильно

        if (current_pin_state && !btn->IsPressed)
        {
          // Кнопку точно нажали
          btn->IsPressed = 1;
          btn->LastTick = current_time; // Засекаем время начала удержания
        }
        else if (!current_pin_state && btn->IsPressed)
        {
          // Кнопку точно отпустили (даже если это произошло быстро)
          btn->IsPressed = 0;
          if ((current_time - btn->LastTick) < BUTTON_LONG_PRESS_TIME)
          {
            btn->Event = BTN_SHORT_PRESS;
          }
        }
      }
    }

    // 2. Логика удержания (длинного нажатия)
    // Работает только когда состояние стабильно (нет дребезга) и кнопка удерживается
    if (btn->IsPressed && !btn->StateChanged)
    {
      // Проверяем на отпускание без дебаунса, если прерывание по какой-то причине пропущено
      if (!current_pin_state)
      {
        btn->IsPressed = 0;
        if ((current_time - btn->LastTick) < BUTTON_LONG_PRESS_TIME)
        {
          btn->Event = BTN_SHORT_PRESS;
        }
      }
      // Проверка удержания по времени
      else if ((current_time - btn->LastTick) >= BUTTON_LONG_PRESS_TIME)
      {
        btn->Event = BTN_LONG_PRESS;
        btn->IsPressed = 0; // Сбрасываем удержание, событие сформировано
      }
    }
  }
}
