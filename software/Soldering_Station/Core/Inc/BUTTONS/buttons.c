/***********************************************************************************************************

  Author:     Am0k
  Github:     https://github.com/Am0k-GIT

***********************************************************************************************************/

#include <BUTTONS/buttons.h>

volatile bool flag_IT_IRQ = false;                                             // флаг прерывания кнопок
volatile bool flag_state_button = false;                                       // флаг нажатия кнопки (после проверки дребезга)
volatile bool flag_longpress = false;                                          // флаг нажатия кнопки (длительного)
volatile uint16_t menu_button_PIN = 0;                                         // нажатая кнопка при внешнем прерывании
volatile GPIO_TypeDef* menu_button_PORT = 0;                                   // порт
volatile IRQn_Type menu_button_IQR = 0;                                        // номер прерывания
volatile uint32_t time_stamp_IRQ = 0;                                          // время последнего внешнего прерывания

char TX_Buffer_[16];

//***** Получаем номер (0-15) из HAL-овского GPIO_Pin *************************
uint8_t HAL_GPIO_GET_REAL_NUMBER(uint16_t pin)
{
    switch (pin)
    {
        case (GPIO_PIN_0):
            return 0;
            break;
        case (GPIO_PIN_1):
            return 1;
            break;
        case (GPIO_PIN_2):
            return 2;
            break;
        case (GPIO_PIN_3):
            return 3;
            break;
        case (GPIO_PIN_4):
            return 4;
            break;
        case (GPIO_PIN_5):
            return 5;
            break;
        case (GPIO_PIN_6):
            return 6;
            break;
        case (GPIO_PIN_7):
            return 7;
            break;
        case (GPIO_PIN_8):
            return 8;
            break;
        case (GPIO_PIN_9):
            return 9;
            break;
        case (GPIO_PIN_10):
            return 10;
            break;
        case (GPIO_PIN_11):
            return 11;
            break;
        case (GPIO_PIN_12):
            return 12;
            break;
        case (GPIO_PIN_13):
            return 13;
            break;
        case (GPIO_PIN_14):
            return 14;
            break;
        case (GPIO_PIN_15):
            return 15;
            break;
    }
}
//*****************************************************************************

//***** Получаем HAL-овский GPIO_Pin  из номера (0-15)  ***********************
uint16_t HAL_GPIO_GET_HAL_PIN(uint8_t pin)
{
    switch (pin)
    {
        case (0):
            return GPIO_PIN_0;
            break;
        case (1):
            return GPIO_PIN_1;
            break;
        case (2):
            return GPIO_PIN_2;
            break;
        case (3):
            return GPIO_PIN_3;
            break;
        case (4):
            return GPIO_PIN_4;
            break;
        case (5):
            return GPIO_PIN_5;
            break;
        case (6):
            return GPIO_PIN_6;
            break;
        case (7):
            return GPIO_PIN_7;
            break;
        case (8):
            return GPIO_PIN_8;
            break;
        case (9):
            return GPIO_PIN_9;
            break;
        case (10):
            return GPIO_PIN_10;
            break;
        case (11):
            return GPIO_PIN_11;
            break;
        case (12):
            return GPIO_PIN_12;
            break;
        case (13):
            return GPIO_PIN_13;
            break;
        case (14):
            return GPIO_PIN_14;
            break;
        case (15):
            return GPIO_PIN_15;
            break;
    }
}
//*****************************************************************************

//***** Callback внешнего прерывания кнопок меню ******************************
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
    menu_button_PORT = __GET_PORT[HAL_GPIO_GET_REAL_NUMBER(GPIO_Pin)];         // получаем порт
    if (menu_button_PORT)                                                      // если порт прописан в __GET_PORT[]
    {
        menu_button_IQR = __GET_IRQ[HAL_GPIO_GET_REAL_NUMBER(GPIO_Pin)];       // получаем прерывание
        HAL_NVIC_DisableIRQ (menu_button_IQR);                                 // сразу же отключаем это прерывание
        menu_button_PIN = GPIO_Pin;                                            // записываем нажатый GPIO_Pin
        flag_IT_IRQ = true;                                                    // поднимаем флаг однократности прерывания
        time_stamp_IRQ = HAL_GetTick ();                                       // записываем время прерывания (для обработки антидребезга)
        __ISB ();
    }
}
//*****************************************************************************

//***** Обработка дребезга кнопок *********************************************
void BUTTON_IRQ_Debounce (void (*callback) (uint16_t key_number, bool longpress))
{
    if (flag_IT_IRQ)                                                           // сработало внешнее прерывание
    {
        bool current_status = !HAL_GPIO_ReadPin (menu_button_PORT, menu_button_PIN);
                                                                               // считываем текущее состояние кнопки, записаннной в прерывании
        uint32_t current_latency = (HAL_GetTick () - time_stamp_IRQ);          // считаем время после срабатывания прерывания
        if (menu_button_PIN && !flag_longpress)                                // если есть записанный ID кнопки (и кнопка не зажата)
        {
            if (!flag_state_button && (current_latency >= BUTTON_LATENCY_DEBOUNCE))
            {                                                                  // флага нажатости еще не было и прошла первичная задержка анти-дребезга
                if (current_status)                                            // если кнопка нажата (нажатие прошло проверку на ложность)
                {
                    flag_state_button = true;                                  // поднимаем флаг нажатости
                }
                else                                                           // если кнопка отжата (нажатие ложное)
                {
                    time_stamp_IRQ = HAL_GetTick ();                           // записываем время
                    flag_state_button = false;                                 // сбрасываем флаг нажатости
                    menu_button_PIN = 0;                                       // сбрасываем ID кнопки
                }
            }
            if (flag_state_button)
            {
                if (!current_status)
                {
                    callback (menu_button_PIN, false);                         // обрабатываем короткое нажатие
                    time_stamp_IRQ = HAL_GetTick ();                           // записываем время
                    flag_state_button = false;                                 // сбрасываем флаг нажатости
                    menu_button_PIN = 0;                                       // сбрасываем ID кнопки
                }
                else
                {
                    if (current_latency >= BUTTON_LATENCY_LONGPRESS)
                    {
                        callback (menu_button_PIN, true);                      // обрабатываем длинное нажатие
                        flag_longpress = true;
                    }
                }
            }
        }
        else                                                                   // данные о ID кнопки уже удалены
        {
            if (flag_state_button)                                             // флаг нажатости еще не был сброшен
            {
                if (!current_status)                                           // кнопка отжата
                {
                    time_stamp_IRQ = HAL_GetTick ();                           // записываем время
                    flag_state_button = false;                                 // сбрасываем флаг нажатости
                    menu_button_PIN = 0;                                       // сбрасываем ID кнопки
                    flag_longpress = false;
                }
            }
            else
            {                                                                  // флаг нажатости был сброшен (и перезаписано время time_stamp_IRQ)
                if (current_latency > BUTTON_LATENCY_DEBOUNCE)                 // задержка антидребезга прошла
                {
                    flag_IT_IRQ = false;                                       // опускаем флаг прерывания
                    for (uint8_t i = 0; i <16; i++)                            // для всех возможных 16 прерываний
                    {
                        if (__GET_PORT[i])                                     // если портдля прерывания объявлен
                        {
                            __HAL_GPIO_EXTI_CLEAR_IT(HAL_GPIO_GET_HAL_PIN(i)); // очищаем бит EXTI_PR (бит прерывания)
                            NVIC_ClearPendingIRQ (__GET_IRQ[i]);               // очищаем бит NVIC_ICPRx (бит очереди)
                            HAL_NVIC_EnableIRQ (__GET_IRQ[i]);                 // включаем внешнее прерывание кнопки меню
                        }
                    }
                }
            }
        }
    }
}
//*****************************************************************************

