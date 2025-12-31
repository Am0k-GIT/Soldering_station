/* USER CODE BEGIN Header */
/***********************************************************************************************************

  Author:     Am0k
  Github:     https://github.com/Am0k-GIT

***********************************************************************************************************/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "BUTTONS/buttons.h"
#include "24CXX/ee24.h"
#include "RingBuffer/ring_buffer.h"
#include "ADCconverter/ADCconverter.h"
#include "ADCconverter/ADCtables.h"

#include "PID/PID.h"
#include "st7735/st7735.h"
#include "st7735/fonts.h"
#include "st7735/images.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile bool flag_changeMenu = true;                                          // menu frame update flag
volatile bool flag_IRON_on = false;                                            // soldering iron enable flag
volatile bool flag_HOTAIR_on = false;                                          // soldering gun enable flag
volatile bool flag_IRON_select_fun = false;                                    // airflow selection flag for encoder
volatile bool flag_HOTAIR_select_fun = false;                                  // airflow selection flag for encoder
volatile bool flag_IT_DMA = false;                                             // DMA interrupt triggered flag
volatile bool flag_IRON_no_tool = true;                                        // no soldering iron flag
volatile bool flag_HOTAIR_no_tool = true;                                      // no soldering gun flag

volatile bool flag_change_IRON_preset = false;                                 // flag for preset loading required
volatile bool flag_change_HOTAIR_preset = false;                               // flag for preset loading required
volatile bool flag_change_IRON_tool = true;                                    // flag for loading tool settings
volatile bool flag_change_HOTAIR_tool = true;                                   // flag for loading tool settings

volatile bool flag_reload_TIM3 = true;                                         // flag for the need to overwrite TIM settings for the encoder
volatile bool flag_reload_TIM4 = true;                                         // flag for the need to overwrite TIM settings for the encoder
volatile bool flag_TIM5_Pulse_Start = false;                                   // TIM5 pulse start flag
volatile bool flag_ADC_Start = false;                                          // ADC start flag
volatile bool flag_ADC_need_restart = false;                                   // the pulse is completed before the end of the ADC operation

volatile bool flag_TIM10_Start = false;                                        // TIM10 start flag
volatile bool flag_IT_SLEEP = false;                                           // sleep mode countdown timer operation flag
volatile bool flag_IT_IRON_SENSOR = false;                                     // soldering iron handle interrupt flag
volatile bool flag_IRON_hold = false;                                          // HOLD mode
volatile bool flag_IRON_off = false;                                           // OFF mode
volatile bool flag_IRON_WAKEUP = false;                                        // need wake up IRON
volatile bool flag_IRON_NTC = false;                                           // IRON have NTC
volatile bool flag_HOTAIR_hold = false;                                        // HOLD mode
volatile bool flag_HOTAIR_off = false;                                         // OFF MODE
volatile bool flag_HOTAIR_WAKEUP = false;                                      // need wake up HOTAIR
volatile bool flag_HOTAIR_NTC = false;                                         // HOTAIR have NTC
volatile bool IRON_ext_sensor = false;
volatile bool flag_update_UI = false;
volatile bool flag_IRON_preheat_start = false;
volatile bool flag_IRON_preheat = false;

volatile bool flag_EEPROM_IRON_SETTINGS_OK = false;
volatile bool flag_EEPROM_HOTAIR_SETTINGS_OK = false;
volatile bool flag_EEPROM_HW_SETTINGS_OK = false;
volatile bool flag_EEPROM_IRON_TOOLS_OK = false;
volatile bool flag_EEPROM_HOTAIR_TOOLS_OK = false;
volatile bool flag_EEPROM_IRON_PRESETS_OK = false;
volatile bool flag_EEPROM_HOTAIR_PRESETS_OK = false;

volatile bool flag_write_IRON_settings = false;                                // flag for the need to write data to EEPROM
volatile bool flag_write_HOTAIR_settings = false;                              // flag for the need to write data to EEPROM
volatile bool flag_write_HW_settings = false;                                  // flag for the need to write data to EEPROM
volatile bool flag_write_IRON_tools = false;                                   // flag for the need to write data to EEPROM
volatile bool flag_write_HOTAIR_tools = false;                                 // flag for the need to write data to EEPROM
volatile bool flag_write_IRON_presets = false;                                 // flag for the need to write data to EEPROM
volatile bool flag_write_HOTAIR_presets = false;                               // flag for the need to write data to EEPROM

uint16_t tim1_ARR = TIM1_COUNT_PER_SEC / TIM1_FREQ;
uint16_t tim2_ARR = TIM2_COUNT_PER_SEC / TIM2_FREQ;
uint16_t tim5_ARR = TIM5_COUNT_PER_SEC / TIM5_FREQ;
uint16_t tim10_ARR = TIM10_COUNT_PER_SEC / TIM10_FREQ;

#ifdef DEBUG_UART
char UART_TX_Buffer[64];
#endif

char TX_Buffer[64];

EE24_HandleTypeDef ee24;                                                       // EEPROM struct

uint8_t menu = 255;                                                            // menu
// 0 - loading screen
// 1 - main screen
// 2 - IRON settings && HOTAIR settings
// 3 - IRON presets
// 4 - HOTAIR presets
// 5 - TOOLS

uint8_t selected_item = 0;                                                     // selected item in any menu
uint16_t selected_sub_menu_settings = 0;
uint8_t IRON_preset_number = 0;
uint8_t HOTAIR_preset_number = 0;
uint8_t IRON_tool_number = 0;
uint16_t selected_tool = 0;
uint8_t calibration_correction = 0;

uint16_t IT_SLEEP_count_IRON = 0;
uint16_t IT_SLEEP_count_HOTAIR = 0;
uint8_t IRON_MAX_ADC_count = 0;
uint8_t HOTAIR_MAX_ADC_count = 0;

volatile uint16_t adc[ADC_CHANNELS] = { 0, 0, 0, 0, 0, 0, 0, 0 };
uint16_t adc_filtered[ADC_CHANNELS] = { 0, 0, 0, 0, 0, 0, 0, 0 };

// [0] - IRON_TC
// [1] - HOTAIR_TC
// [2] - IRON_NTC
// [3] - HOTAIR_NTC
// [4] - ONBOARD_NTC
// [5] - 24V
// [6] - VrefInt
// [7] - IRON_amperage

RING_buffer_t ADC_values[ADC_CHANNELS];

float temperature_IRON_TC = 0;
float temperature_HOTAIR_TC = 0;
float temperature_ONBOARD_NTC = 0;
float temperature_IRON_NTC = 0;
float temperature_HOTAIR_NTC = 0;
float voltage24 = 0;
float K_adc = 1;
float IRON_amperage = 0;

uint16_t IRON_set_temp = 0;
uint16_t IRON_set_temp_RR = 0;
float IRON_real_temp = 0;
uint16_t IRON_blow = 0;

uint16_t HOTAIR_set_temp = 0;
uint16_t HOTAIR_set_temp_RR = 0;
float HOTAIR_real_temp = 0;
uint16_t HOTAIR_blow = 0;

uint32_t time_stamp = 0;
uint32_t time_prev = 0;
uint32_t time_curr = 0;
uint16_t FPS = 0;

typedef struct
{
  uint16_t min_temp;
  uint16_t max_temp;
  uint16_t step_temp;
  uint16_t min_flow;
  uint16_t max_flow;
  uint16_t step_flow;
  uint16_t ext_NTC;
  uint16_t sensor;
  uint16_t sleep_timer;
} settings_t;

typedef struct
{
  uint16_t NTC;
  uint16_t ZERO_CURRENT;
  uint16_t K_CURRENT;
  uint16_t K_VOLTAGE;
  uint16_t K_INT_NTC;
  uint16_t K_IRON_NTC;
  uint16_t K_HOTAIR_NTC;
  uint16_t time_off;
  uint16_t overcurrent_protection;
} hw_settings_t;

typedef struct
{
  uint16_t temp;
  uint16_t flow;
} preset_t;

typedef uint16_t raw_value_t;                                                  // ADC value
typedef int16_t temp_value_t;                                                  // temperature value

typedef struct
{
  temp_value_t raw_value;
  raw_value_t temp_value;
} calibration_table_t;

typedef struct
{
  uint16_t P;
  uint16_t I;
  uint16_t D;
  calibration_table_t calibration_table[5];
} tools_t;

settings_t IRON;
settings_t HOTAIR;
hw_settings_t HW_SET;

settings_t DEF_IRON = {
DEF_IRON_MIN_TEMP,
DEF_IRON_MAX_TEMP,
DEF_IRON_STEP_TEMP,
DEF_IRON_MIN_FLOW,
DEF_IRON_MAX_FLOW,
DEF_IRON_STEP_FLOW,
DEF_IRON_EXT_NTC,
DEF_IRON_EXT_SENSOR,
DEF_IRON_SLEEP_TIMER };

settings_t DEF_HOTAIR = {
DEF_HOTAIR_MIN_TEMP,
DEF_HOTAIR_MAX_TEMP,
DEF_HOTAIR_STEP_TEMP,
DEF_HOTAIR_MIN_FLOW,
DEF_HOTAIR_MAX_FLOW,
DEF_HOTAIR_STEP_FLOW,
DEF_HOTAIR_EXT_NTC,
DEF_HOTAIR_EXT_SENSOR,
DEF_HOTAIR_SLEEP_TIMER };

hw_settings_t DEF_HW_SET = {
DEF_NTC,
DEF_ZERO_CURRENT,
DEF_K_CURRENT,
DEF_K_VOLTAGE,
DEF_K_INT_NTC,
DEF_K_IRON_NTC,
DEF_K_HOTAIR_NTC,
DEF_TIME_OFF,
DEF_OVERCURRENT_PROTECTION,
};

calibration_table_t DEF_CAL_TABLE_IRON[5] = { { DEF_IRON_TOOL_ADC0, DEF_IRON_TOOL_T0 }, { DEF_IRON_TOOL_ADC1,
DEF_IRON_TOOL_T1 }, { DEF_IRON_TOOL_ADC2, DEF_IRON_TOOL_T2 }, { DEF_IRON_TOOL_ADC3, DEF_IRON_TOOL_T3 }, {
DEF_IRON_TOOL_ADC4, DEF_IRON_TOOL_T4 }, };

calibration_table_t DEF_CAL_TABLE_HOTAIR[5] = { { DEF_HOTAIR_TOOL_ADC0, DEF_HOTAIR_TOOL_T0 }, { DEF_HOTAIR_TOOL_ADC1,
DEF_HOTAIR_TOOL_T1 }, { DEF_HOTAIR_TOOL_ADC2, DEF_HOTAIR_TOOL_T2 }, { DEF_HOTAIR_TOOL_ADC3, DEF_HOTAIR_TOOL_T3 }, { DEF_HOTAIR_TOOL_ADC4,
        DEF_HOTAIR_TOOL_T4 }, };

preset_t IRON_presets[IRON_PRESETS];
preset_t HOTAIR_presets[HOTAIR_PRESETS];

preset_t DEF_IRON_presets[IRON_PRESETS] = { { 250, 70 }, { 270, 80 }, { 320, 100 }, };

preset_t DEF_HOTAIR_presets[HOTAIR_PRESETS] = { { 150, 100 }, { 320, 60 }, { 350, 100 }, };

tools_t IRON_tools[IRON_TOOLS] = { };

tools_t HOTAIR_tools;

PID_t PID_IRON;
PID_t PID_HOTAIR;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void TIMers_Init (void);
void TIMers_Update (void);
void PID_REG_IRON (void);
void PID_REG_HOTAIR (void);
void ADC_Work (void);
void Check_Protection (void);
void Check_Sleep (void);
void Check_Tools_Change (void);

void ST7735_Update (void);
void ST7735_prepareFrame (void);
void ST7735_prepareBotMenu_v0 (void);
void ST7735_prepareBotMenu_v1 (void);
void ST7735_prepareBotMenu_v2 (void);
void ST7735_prepareMenu_0 (void);
void ST7735_prepareMenu_1 (void);
void ST7735_prepareMenu_2 (void);
void ST7735_prepareMenu_3 (uint8_t max_presets);
void ST7735_prepareMenu_5 (void);
void ST7735_showMenu_0 (void);
void ST7735_showMenu_1 (void);
void ST7735_showMenu_2_p1 (void);
void ST7735_showMenu_2_p2 (settings_t *set);
void ST7735_showMenu_2_p3 (void);
void ST7735_showMenu_3 (preset_t *preset, uint8_t max_presets, settings_t *set);
void ST7735_showMenu_5_p1 (void);
void ST7735_showMenu_5_p2 (tools_t *tool);
void ST7735_showBar (uint16_t x, uint16_t y, uint16_t length, uint16_t width, float percent);

bool EEPROM_Init (void);
void EEPROM_Show (void);
void EEPROM_Read_All (void);
void EEPROM_Write_All (void);
void EEPROM_RESET (void);

void Button_React (uint16_t key_number, bool longpress);

void encoder_uploader (TIM_HandleTypeDef *htim, bool *flag, uint16_t *set, uint16_t min, uint16_t max, uint8_t step);
float get_K (uint16_t K);
uint32_t approximateT (uint16_t ADC_4, uint16_t ADC_3, uint16_t ADC_2, uint16_t T_3, uint16_t T_2);
float get_NTC_T (uint16_t table, float K, uint16_t adc);

void TEST_INFO ();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART6_UART_Init();
  MX_TIM5_Init();
  MX_SPI1_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin (PS_ON_GPIO_Port, PS_ON_Pin, 1);
  HAL_GPIO_WritePin (SSR_ON_GPIO_Port, SSR_ON_Pin, 1);

  ST7735_Init ();

  #ifdef DEBUG_UART
  snprintf (UART_TX_Buffer, 64, "Start loading Amok Soldering station.\n");
  HAL_UART_Transmit (UART_HEANDLER, (uint8_t*) UART_TX_Buffer, strlen (UART_TX_Buffer), 10);
  snprintf (UART_TX_Buffer, 64, "Firmware version: %6i .\n", VERSION);
  HAL_UART_Transmit (UART_HEANDLER, (uint8_t*) UART_TX_Buffer, strlen (UART_TX_Buffer), 10);
  #endif

  if (EEPROM_Init ())
  {
    EEPROM_Read_All ();
    menu = 0;
    #ifdef DEBUG_UART
    EEPROM_Show ();
    #endif
  }
  else
  {
    menu = 1;
    #ifdef DEBUG_UART
    snprintf (UART_TX_Buffer, 64, "Cant init EEPROM!\n");
    HAL_UART_Transmit (UART_HEANDLER, (uint8_t*) UART_TX_Buffer, strlen (UART_TX_Buffer), 10);
    #endif
  }

  TIMers_Init ();

  for (uint8_t i = 0; i < ADC_CHANNELS; i++)
  {
    RING_BUF_Init (&ADC_values[i], ADC_BUFFER_SIZE);
  }

  time_stamp = HAL_GetTick ();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    BUTTON_IRQ_Debounce (Button_React);

    TIMers_Update ();

    ADC_Work ();

    ST7735_Update ();

    Check_Protection ();

    Check_Sleep ();

    Check_Tools_Change ();

    EEPROM_Write_All ();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// ****************** Callback по завершению работы АЦП ***********************
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)                                                  // по завершению работы ADC
  {
    HAL_ADC_Stop_DMA (&hadc1);                                                 // останавливаем работу ADC в режиме DMA
    flag_ADC_Start = false;                                                    // сбрасываем флаг работы ADC

    if (flag_TIM5_Pulse_Start)                                                 // если ADC запущен в начале периода PWM
    {
      if (!flag_ADC_need_restart)                                              // если не произошел конец импульса до конца работы ADC
        RING_BUF_Push (&ADC_values[7], adc[7]);                                // отправляем в кольцевой буфер значение ADC[7] (ток жала)
    }
    else                                                                       // если ADC запущен по окончанию периода PWM
    {
      if (adc[0] > ADC_MAX_VALUE)                                              // проверяем подключение термопары паяльника
      {
        IRON_MAX_ADC_count++;
      }
      else                                                                     // если термопара подключена
      {
        flag_IRON_no_tool = false;
        IRON_MAX_ADC_count = 0;
        RING_BUF_Push (&ADC_values[0], adc[0]);                                // отправляем в кольцевой буфер значения ADC[0]
      }

      if (adc[1] > ADC_MAX_VALUE)                                              // проверяем подключение термопары фена
      {
        HOTAIR_MAX_ADC_count++;
      }
      else                                                                     // если термопара подключена
      {
        flag_HOTAIR_no_tool = false;
        HOTAIR_MAX_ADC_count = 0;
        RING_BUF_Push (&ADC_values[1], adc[1]);                                // отправляем в кольцевой буфер значение ADC[1]
      }

      if (adc[2] > ADC_MAX_VALUE)                                              // проверяем подключение NTC резистора в рукоятке паяльника
      {
        flag_IRON_NTC = false;
      }
      else                                                                     // если NTC резистор подключен
      {
        flag_IRON_NTC = true;
        RING_BUF_Push (&ADC_values[2], adc[2]);                                // отправляем в кольцевой буфер значение ADC[2]
      }

      if (adc[3] > ADC_MAX_VALUE)                                              // проверяем подключение NTC резистора в рукоятке фена
      {
        flag_HOTAIR_NTC = false;
      }
      else                                                                     // если NTC резистор подключен
      {
        flag_HOTAIR_NTC = true;
        RING_BUF_Push (&ADC_values[3], adc[3]);                                // отправляем в кольцевой буфер значение ADC[3]
      }

      for (uint8_t i = 4; i < ADC_CHANNELS - 1; i++)                           // для каналов ADC[4-6]
      {
        RING_BUF_Push (&ADC_values[i], adc[i]);                                // отправляем в кольцевой буфер значения ADC[4-6]
      }

      HAL_TIM_PWM_Start_IT (&htim5, TIM_CHANNEL_4);                            // запускаем PWM TIM5
      __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);

      HAL_TIM_Base_Start_IT (&htim10);                                         // запускаем TIM10 (для получения тока жала)
      flag_TIM5_Pulse_Start = true;                                            // поднимаем флаг начала периода PWM
      flag_ADC_Start = true;                                                   // поднимаем флаг начала работы ADC
      flag_ADC_need_restart = false;                                           // сбрасываем флаг слишком короткого импульса
    }

    flag_IT_DMA = true;                                                        // поднимаем флаг окончания работы прерывания DMA
  }
}
// ****************************************************************************

// ****************** Callback по завершению положительного импульса TIM5 *****
void HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM5)                                                  // по окончанию импульса PWM TIM5
  {
    if (flag_ADC_Start)                                                        // если ADC сейчас запущено (еще не завершило работу)
    {
      flag_ADC_need_restart = true;                                            // поднимаем флаг слишком короткого импульса
    }
  }
}
// ****************************************************************************

// ****************** Callback TIM5, TIM10, TIM11 *****************************
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)                                                  // по завершению периода TIM2
  {
    PID_REG_HOTAIR ();
  }

  if (htim->Instance == TIM5)                                                  // по завершению периода TIM5
  {
    HAL_TIM_PWM_Stop_IT (&htim5, TIM_CHANNEL_4);                               // отключаем PWM
    HAL_TIM_Base_Start_IT (&htim10);                                           // запускаем таймер задержки перед измерением
    flag_TIM5_Pulse_Start = false;
    flag_ADC_Start = true;
    PID_REG_IRON ();
  }

  if (htim->Instance == TIM10)                                                 // по завершению периода TIM10
  {
    HAL_TIM_Base_Stop_IT (&htim10);                                            // отключаем TIM10
    HAL_ADC_Start_DMA (&hadc1, (uint32_t*) &adc, ADC_CHANNELS);                // запускаем ADC в режиме DMA
  }

  if (htim->Instance == TIM11)                                                 // по завершению периода TIM11
  {
    flag_update_UI = true;
    IRON_ext_sensor = HAL_GPIO_ReadPin (EXT_SENSOR_GPIO_Port, EXT_SENSOR_Pin); // проверяем подставку
    IT_SLEEP_count_IRON++;                                                     // увеличиваем счетчик сна для IRON
    IT_SLEEP_count_HOTAIR++;                                                   // увеличиваем счетчик сна для HOTAIR
  }
}
// ****************************************************************************

// ****************** �?нициализация таймеров *********************************
void TIMers_Init (void)
{
  // *** Air blow TIM
  TIM1->PSC = APB2_TIM_FREQ / TIM1_COUNT_PER_SEC - 1;
  TIM1->ARR = tim1_ARR;
  TIM1->EGR |= TIM_EGR_UG;

  // *** HOTAIR PWM TIM
  TIM2->PSC = APB1_TIM_FREQ / TIM2_COUNT_PER_SEC - 1;
  TIM2->ARR = tim2_ARR;
  TIM2->EGR |= TIM_EGR_UG;

  // *** IRON PWM TIM
  TIM5->PSC = APB1_TIM_FREQ / TIM5_COUNT_PER_SEC - 1;
  TIM5->ARR = tim5_ARR;
  TIM5->EGR |= TIM_EGR_UG;

  // *** MEASUREMENT DELAY TIM
  TIM10->PSC = APB2_TIM_FREQ / TIM10_COUNT_PER_SEC - 1;
  TIM10->ARR = MEASUREMENT_DELAY;
  TIM10->EGR |= TIM_EGR_UG;

  // *** SLEEP COUNT INCREASE TASK
  TIM11->PSC = APB2_TIM_FREQ / TIM11_COUNT_PER_SEC - 1;
  TIM11->ARR = TIM11_COUNT_PER_SEC / TIM11_FREQ;

  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;

  TIM2->CCR1 = 0;

  TIM5->CCR4 = 0;

  HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_2);

  //HAL_TIM_PWM_Start (&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT (&htim2, TIM_CHANNEL_1);                                // Запускаем PWM TIM5
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);

  HAL_TIM_Encoder_Start (&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start (&htim4, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start_IT (&htim5, TIM_CHANNEL_4);                                // Запускаем PWM TIM5
  __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);

  PID_Init (&PID_IRON, false);
  PID_SetLimits (&PID_IRON, 0, TIM5->ARR);
  PID_SetSync (&PID_IRON, round(1000 / TIM5_FREQ) + 1);

  PID_Init (&PID_HOTAIR, false);
  PID_SetLimits (&PID_HOTAIR, 0, TIM2->ARR);
  PID_SetSync (&PID_HOTAIR, round(1000 / TIM2_FREQ) + 1);

  HAL_TIM_Base_Start_IT (&htim11);                                             // прерывания TIM11
}
// ****************************************************************************

// ****************** Управление таймерами ************************************
void TIMers_Update (void)
{
  if (flag_IRON_on)
  {
    TIM1->CCR1 = tim1_ARR * IRON_blow / 100;
  }
  else
  {
    TIM5->CCR4 = 0;
    TIM1->CCR1 = 0;
  }

  if (flag_HOTAIR_on)
  {
    TIM1->CCR2 = tim1_ARR * HOTAIR_blow / 100;
  }
  else
  {
    TIM2->CCR1 = 0;
    if (HOTAIR_real_temp > HOTAIR_TEMP_COOLDOWN)
    {
      TIM1->CCR2 = tim1_ARR * HOTAIR_BLOW_COOLDOWN / 100;
    }
    else
    {
      TIM1->CCR2 = 0;
    }
  }
}
// ****************************************************************************

// ****************** Работа PID регулятора ***********************************
void PID_REG_IRON (void)
{
  if (flag_IRON_preheat_start)
  {
    flag_IRON_preheat_start = false;
    flag_IRON_preheat = true;
    PID_SetLimits (&PID_IRON, 0, (TIM5->ARR) * IRON_PREHEAT_POWER / 100);
    PID_Set_K (&PID_IRON, (float)PID_P_IRON_SCALLING * IRON_PREHEAT_P_FACTOR * IRON_tools[IRON_tool_number].P, 0, 0, PID_I_RANGE);
    PID_Restart (&PID_IRON);
  }
  if (flag_IRON_preheat && (IRON_real_temp + IRON_PREHEAT_RANGE > IRON_set_temp))
  {
    flag_IRON_preheat = false;
    PID_SetLimits (&PID_IRON, 0, TIM5->ARR);
    PID_Set_K (&PID_IRON, (float)PID_P_IRON_SCALLING * IRON_tools[IRON_tool_number].P, (float)PID_I_IRON_SCALLING * IRON_tools[IRON_tool_number].I,
               (float)PID_D_IRON_SCALLING * IRON_tools[IRON_tool_number].D, PID_I_RANGE);
    PID_Restart (&PID_IRON);
  }

  if (flag_IRON_on)
  {
    if (flag_IRON_hold)
    {
      PID_SetValue (&PID_IRON, IRON_HOLD_TEMP);
    }
    else
    {
      PID_SetValue (&PID_IRON, IRON_set_temp);
    }
    TIM5->CCR4 = PID_GetResultSync (&PID_IRON, IRON_real_temp);

    #ifdef DEBUG_UART
    snprintf (UART_TX_Buffer, 64, "I S%1i %03iC: P%04i I%04i D%04i\n", flag_TIM5_Pulse_Start, (uint16_t)(round(IRON_real_temp)), PID_Get_P(&PID_IRON), PID_Get_I(&PID_IRON), PID_Get_D(&PID_IRON));
    HAL_UART_Transmit (UART_HEANDLER, (uint8_t*) UART_TX_Buffer, strlen (UART_TX_Buffer), 10);
    #endif
  }
  else
  {
    TIM5->CCR4 = 0;
    PID_Restart (&PID_IRON);
  }
}
// ****************************************************************************

// ****************** Работа PID регулятора ***********************************
void PID_REG_HOTAIR (void)
{

  if (flag_HOTAIR_on)
  {
    if (flag_HOTAIR_hold)
    {
      PID_SetValue (&PID_HOTAIR, HOTAIR_HOLD_TEMP);
    }
    else
    {
      PID_SetValue (&PID_HOTAIR, HOTAIR_set_temp);
    }
    TIM2->CCR1 = PID_GetResultSync (&PID_HOTAIR, HOTAIR_real_temp);

    #ifdef DEBUG_UART
    snprintf (UART_TX_Buffer, 64, "A %03iC: P%04i I%04i D%04i\n", (uint16_t)(round(HOTAIR_real_temp)), PID_Get_P(&PID_HOTAIR), PID_Get_I(&PID_HOTAIR), PID_Get_D(&PID_HOTAIR));
    HAL_UART_Transmit (UART_HEANDLER, (uint8_t*) UART_TX_Buffer, strlen (UART_TX_Buffer), 10);
    #endif
  }
  else
  {
    TIM2->CCR1 = 0;
    PID_Restart (&PID_HOTAIR);
  }
}
// ****************************************************************************

// ****************** Обработка ADC *******************************************
void ADC_Work (void)
{
  if (flag_IT_DMA)                                                             // если АЦП отработало
  {
    flag_IT_DMA = false;                                                       // опускаем флаг

    for (uint8_t i = 0; i < ADC_CHANNELS; i++)                                 // для всех каналов АЦП
    {
      adc_filtered[i] = RING_BUF_getFiltered (&ADC_values[i], PRECISION);      // получаем отфильтрованные значения
    }

    uint32_t vrefintCal = (*VREFINT_CAL_ADDR);                                 // получаем калибровочное значение АЦП
    K_adc = (float) vrefintCal / adc_filtered[6];                              // считаем коэффициент поправки
    IRON_amperage = (adc_filtered[7] - HW_SET.ZERO_CURRENT) * ACS712_K * get_K(HW_SET.K_CURRENT);
    voltage24 = GetVoltageRVD (K_adc * adc_filtered[5], 4095, 3.3, R314, R315) * get_K(HW_SET.K_VOLTAGE);

    temperature_ONBOARD_NTC =  get_NTC_T (HW_SET.NTC, get_K(HW_SET.K_INT_NTC), adc_filtered[4]);

    temperature_IRON_TC = ADC_converter (adc_filtered[0], IRON_tools[IRON_tool_number].calibration_table,
                                         TABLE_SIZE(IRON_tools[IRON_tool_number].calibration_table));
    IRON_real_temp = temperature_IRON_TC;                                      // подсчитываем температуру паяльника

    if (IRON.ext_NTC && flag_IRON_NTC)
    {
      temperature_IRON_NTC =  get_NTC_T (IRON.ext_NTC, get_K(HW_SET.K_IRON_NTC), adc_filtered[2]);
      IRON_real_temp += temperature_IRON_NTC;
    }
    else
    {
      IRON_real_temp += temperature_ONBOARD_NTC;
    }

    temperature_HOTAIR_TC = ADC_converter (adc_filtered[1], HOTAIR_tools.calibration_table, TABLE_SIZE(HOTAIR_tools.calibration_table));
    HOTAIR_real_temp = temperature_HOTAIR_TC;                                  // подсчитываем температуру фена

    if (HOTAIR.ext_NTC && flag_HOTAIR_NTC)
    {
      temperature_HOTAIR_NTC =  get_NTC_T (HOTAIR.ext_NTC, get_K(HW_SET.K_HOTAIR_NTC), adc_filtered[3]);
      HOTAIR_real_temp += temperature_HOTAIR_NTC;
    }
    else
    {
      HOTAIR_real_temp += temperature_ONBOARD_NTC;
    }
  }
}
//*****************************************************************************

//******************* Проверка защитных параметров ****************************
void Check_Protection (void)
{
  if ((IRON_real_temp > (IRON.max_temp + IRON_OVERHEAT_PROTECTION)) || (HOTAIR_real_temp > (HOTAIR.max_temp + HOTAIR_OVERHEAT_PROTECTION)))
  {                                                                            // проверка перегрева
    HAL_GPIO_WritePin (PS_ON_GPIO_Port, PS_ON_Pin, 0);                         // отключаем паяльную станцию
  }

  if (HW_SET.overcurrent_protection && (IRON_amperage > HW_SET.overcurrent_protection))
  {                                                                            // проверка максимамльного тока
    flag_IRON_on = false;
  }

  if (flag_HOTAIR_off && flag_IRON_off)                                        // флаги отключения фена и паяльника подняты
    HAL_GPIO_WritePin (PS_ON_GPIO_Port, PS_ON_Pin, 0);                         // отключаем паяльную станцию

  if (IRON_MAX_ADC_count > ADC_BUFFER_SIZE)
  {
    flag_IRON_no_tool = true;
    flag_IRON_on = false;
  }
  if (HOTAIR_MAX_ADC_count > ADC_BUFFER_SIZE)
  {
    flag_HOTAIR_no_tool = true;
    flag_HOTAIR_on = false;
  }

  if (flag_HOTAIR_no_tool)                                                     // проверка на подключение ручки фена
  {
    HAL_GPIO_WritePin (SSR_ON_GPIO_Port, SSR_ON_Pin, 0);                       // отключаем pin SSR_ON
  }
  else
  {
    HAL_GPIO_WritePin (SSR_ON_GPIO_Port, SSR_ON_Pin, 1);                       // включаем pin SSR_ON
  }
}
//*****************************************************************************

// ****************** Проверка засыпаний / пробуждений ************************
void Check_Sleep (void)
{
  if (!IRON.sensor && IRON_ext_sensor)                                         // сенсор в рукоятке отключен и паяльник не в подставке
  {
    flag_IRON_WAKEUP = true;                                                   // поднимаем флаг пробуждения
  }

  if (flag_IRON_WAKEUP)                                                        // если флаг пробуждения паяльника поднят
  {                                                                            // (от сенсора в ручке или снятого с подставки паяльника)
    IT_SLEEP_count_IRON = 0;
    if (flag_IRON_hold)                                                        // если паяльник в режиме HOLD
    {
      flag_IRON_hold = false;                                                  // снимаем режим HOLD
      if (flag_IRON_off)                                                       // если паяльник в режиме OFF
      {
        flag_IRON_off = false;                                                 // отключаем режим OFF
      }
      else                                                                     // если паяльник не в режиме OFF
      {
        if (flag_IRON_on)
          flag_IRON_preheat_start = true;                                      // запускаем преднагрев
      }
    }
    flag_IRON_WAKEUP = false;                                                  // снимаем флаг пробуждения
  }

  if (flag_IRON_hold)                                                          // если паяльник в режиме HOLD
  {
    if (flag_IRON_off)                                                         // если паяльник в режиме OFF
    {

    }
    else                                                                       // если паяльник не в режиме OFF
    {
      if (IT_SLEEP_count_IRON >= HW_SET.time_off * TIM11_FREQ)                 // таймер отключения достиг своего значения
      {
        flag_IRON_off = true;                                                  // устанавливаем режим OFF
        flag_IRON_on = false;                                                  // сбрасываем флаг включения
        if (menu == 1)
          flag_reload_TIM3 = true;                                             // перезагружаем TIM3 (для доступа к установленной температуре)
       }
    }
  }
  else                                                                         // если паяльник не в режиме HOLD
  {
    if (IT_SLEEP_count_IRON >= IRON.sleep_timer * TIM11_FREQ)                  // если период времени для засыпания окончен
    {
      flag_IRON_hold = true;                                                   // устанавливаем режим HOLD
      IT_SLEEP_count_IRON = 0;                                                 // сброс таймера засыпания

      if (menu == 1)
        flag_reload_TIM3 = true;                                               // перезагружаем TIM3 (для доступа к установленной температуре)
    }
  }

  if (flag_HOTAIR_WAKEUP)                                                      // если флаг пробуждения фена поднят
  {                                                                            // (фен снят с подставки)
    IT_SLEEP_count_HOTAIR = 0;
    if (flag_HOTAIR_hold)                                                      // если фен в режиме HOLD
    {
      flag_HOTAIR_hold = false;                                                // снимаем режим HOLD
      if (flag_HOTAIR_off)                                                     // если фен в режиме OFF
      {
        flag_HOTAIR_off = false;                                               // отключаем режим OFF
      }
      else                                                                     // если фен не в режиме OFF
      {

      }
    }
    flag_HOTAIR_WAKEUP = false;                                                // снимаем флаг пробуждения
  }

  if (HOTAIR.sensor)
  {
    if (flag_HOTAIR_NTC)                                                       // если сенсор включен и фен не в подставке
    {
      flag_HOTAIR_WAKEUP = true;                                               // поднимаем флаг пробуждения
    }

    if (flag_HOTAIR_hold)                                                      // если фен в режиме HOLD
    {
      if (flag_HOTAIR_off)                                                     // если фен в режиме OFF
      {

      }
      else                                                                     // если фен не в режиме OFF
      {
        if (IT_SLEEP_count_HOTAIR >= HW_SET.time_off * TIM11_FREQ)             // таймер отключения достиг своего значения
        {
          flag_HOTAIR_off = true;                                              // устанавливаем режим OFF
          flag_HOTAIR_on = false;                                              // сбрасываем флаг включения
          if (menu == 1)
            flag_reload_TIM4 = true;                                           // перезагружаем TIM4 (для доступа к установленной температуре)
        }
      }
    }
    else                                                                       // если фен не в режиме HOLD
    {
      if (IT_SLEEP_count_HOTAIR >= HOTAIR.sleep_timer * TIM11_FREQ)            // если период времени для засыпания окончен
      {
        flag_HOTAIR_hold = true;                                               // устанавливаем режим HOLD
        IT_SLEEP_count_HOTAIR = 0;                                             // сброс таймера засыпания

        if (menu == 1)
          flag_reload_TIM4 = true;                                             // перезагружаем TIM4 (для доступа к установленной температуре)
      }
    }
  }
}
//*****************************************************************************

// ****************** Проверка на переключение инструмента ********************
void Check_Tools_Change()
{
  if (flag_change_IRON_tool)
  {
    PID_Set_K (&PID_IRON, (float)PID_P_IRON_SCALLING * IRON_tools[IRON_tool_number].P, (float)PID_I_IRON_SCALLING * IRON_tools[IRON_tool_number].I,
               (float)PID_D_IRON_SCALLING * IRON_tools[IRON_tool_number].D , PID_I_RANGE);
    PID_Restart (&PID_IRON);
    flag_change_IRON_tool = false;
  }

  if (flag_change_HOTAIR_tool)
  {
    PID_Set_K (&PID_HOTAIR, (float)PID_P_HOTAIR_SCALLING * HOTAIR_tools.P, (float)PID_I_HOTAIR_SCALLING * HOTAIR_tools.I,
               (float)PID_D_HOTAIR_SCALLING * HOTAIR_tools.D, PID_I_RANGE);
    PID_Restart (&PID_HOTAIR);
    flag_change_HOTAIR_tool = false;
  }
}
//*****************************************************************************

// ****************** Обновление экрана ***************************************
void ST7735_Update (void)
{
  if (flag_update_UI)
  {
  if (flag_changeMenu)
    ST7735_prepareFrame ();

  switch (menu)
  {
    case (0):                                                                  // Loading screen
      if (flag_changeMenu)
      {
        ST7735_prepareBotMenu_v0 ();
        ST7735_prepareMenu_0 ();
        flag_changeMenu = false;
      }

      ST7735_showMenu_0 ();

      break;

    case (1):                                                                  // Main menu
      if (flag_changeMenu)
      {
        ST7735_prepareBotMenu_v2 ();
        ST7735_prepareMenu_1 ();
        flag_changeMenu = false;
      }

      ST7735_showMenu_1 ();

      break;

    case (2):                                                                  // IRON settings && HOTAIR settings
      if (flag_changeMenu)
      {
        ST7735_prepareBotMenu_v1 ();
        flag_changeMenu = false;
      }
      ST7735_showMenu_2_p1 ();

      break;

    case (3):
      if (flag_changeMenu)
      {
        ST7735_prepareBotMenu_v1 ();
        ST7735_prepareMenu_3 (IRON_PRESETS);
        flag_changeMenu = false;
      }

      snprintf (TX_Buffer, 64, "IRON PRESETS:");
      ST7735_WriteString (14, 3, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

      ST7735_showMenu_3 (&IRON_presets, IRON_PRESETS, &IRON);

      break;

    case (4):
      if (flag_changeMenu)
      {
        ST7735_prepareBotMenu_v1 ();
        ST7735_prepareMenu_3 (HOTAIR_PRESETS);
        flag_changeMenu = false;
      }

      snprintf (TX_Buffer, 64, "HOTAIR PRESETS:");
      ST7735_WriteString (14, 3, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

      ST7735_showMenu_3 (&HOTAIR_presets, HOTAIR_PRESETS, &HOTAIR);

      break;

    case (5):
      if (flag_changeMenu)
      {
        selected_tool = 0;
        ST7735_prepareBotMenu_v1 ();
        ST7735_prepareMenu_5 ();
        flag_changeMenu = false;
      }

      ST7735_showMenu_5_p1 ();

      break;
  }

  flag_update_UI = false;
  }
}
//*****************************************************************************

//*****************************************************************************
void ST7735_prepareFrame (void)
{
  ST7735_FillScreen (ST7735_BLACK);
  for (int x = 0; x < ST7735_WIDTH; x++)
  {
    ST7735_DrawPixel (x, 0, ST7735_RED);
    ST7735_DrawPixel (x, ST7735_HEIGHT - 1, ST7735_RED);
  }
  for (int y = 0; y < ST7735_HEIGHT; y++)
  {
    ST7735_DrawPixel (0, y, ST7735_RED);
    ST7735_DrawPixel (ST7735_WIDTH - 1, y, ST7735_RED);
  }
}
//*****************************************************************************

//*****************************************************************************
void ST7735_prepareBotMenu_v0 (void)
{
  for (int x = 0; x < ST7735_WIDTH; x++)
  {
    ST7735_DrawPixel (x, 114, ST7735_RED);
  }

  snprintf (TX_Buffer, 64, "             RST  EXIT");
  ST7735_WriteString (4, 117, TX_Buffer, Font_7x10, ST7735_COLOR565(100, 100, 60), ST7735_BLACK);
}
//*****************************************************************************

//*****************************************************************************
void ST7735_prepareBotMenu_v1 (void)
{
  for (int x = 0; x < ST7735_WIDTH; x++)
  {
    ST7735_DrawPixel (x, 114, ST7735_RED);
  }

  snprintf (TX_Buffer, 64, "BACK  NEXT  SAVE  EXIT");
  ST7735_WriteString (4, 117, TX_Buffer, Font_7x10, ST7735_COLOR565(100, 100, 60), ST7735_BLACK);
}
//*****************************************************************************

//*****************************************************************************
void ST7735_prepareBotMenu_v2 (void)
{
  for (int x = 0; x < ST7735_WIDTH; x++)
  {
    ST7735_DrawPixel (x, 104, ST7735_RED);
  }

  snprintf (TX_Buffer, 64, "PRST  TOOL  SETT  PRST");
  ST7735_WriteString (4, 107, TX_Buffer, Font_7x10, ST7735_COLOR565(100, 100, 60), ST7735_BLACK);

  snprintf (TX_Buffer, 64, "CONF  CONF   OFF  CONF");
  ST7735_WriteString (4, 117, TX_Buffer, Font_7x10, ST7735_COLOR565(100, 100, 60), ST7735_BLACK);
}
//*****************************************************************************

//*****************************************************************************
void ST7735_prepareMenu_0 (void)
{
  snprintf (TX_Buffer, 64, "LOADING SETTINGS");
  ST7735_WriteString (14, 3, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "IRON SETT.   -");
  ST7735_WriteString (14, 14, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "HOTAIR SETT. -");
  ST7735_WriteString (14, 25, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "HW SETT.     -");
  ST7735_WriteString (14, 36, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "IRON TOOLS   -");
  ST7735_WriteString (14, 47, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "HOTAIR TOOLS -");
  ST7735_WriteString (14, 58, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "IRON PRES.   -");
  ST7735_WriteString (14, 69, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "HOTAIR PRES. -");
  ST7735_WriteString (14, 80, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "VERSION: ");
  ST7735_WriteString (14, 102, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);
}
//*****************************************************************************

//*****************************************************************************
void ST7735_prepareMenu_1 (void)
{
  for (int x = 0; x < ST7735_WIDTH; x++)
  {
    ST7735_DrawPixel (x, 78, ST7735_RED);
  }

  ST7735_DrawImage (1, 1, 20, 40, (uint16_t*) IRON_20x40);
  ST7735_DrawImage (80, 1, 20, 40, (uint16_t*) SMD_20x40);

  snprintf (TX_Buffer, 64, "'C");
  ST7735_WriteString (56, 2, TX_Buffer, Font_11x18, ST7735_YELLOW, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "F   %%");
  ST7735_WriteString (23, 20, TX_Buffer, Font_11x18, ST7735_CYAN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "'");
  ST7735_WriteString (58, 48, TX_Buffer, Font_16x26, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "'C");
  ST7735_WriteString (136, 2, TX_Buffer, Font_11x18, ST7735_YELLOW, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "F   %%");
  ST7735_WriteString (103, 20, TX_Buffer, Font_11x18, ST7735_CYAN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "'");
  ST7735_WriteString (138, 48, TX_Buffer, Font_16x26, ST7735_GREEN, ST7735_BLACK);
}
//*****************************************************************************

//*****************************************************************************
void ST7735_prepareMenu_3 (uint8_t max_presets)
{
  for (uint8_t i = 0; i < max_presets; i++)

  {
    snprintf (TX_Buffer, 64, "#%1i", i + 1);
    ST7735_WriteString (3, 15 + i * 11, TX_Buffer, Font_7x10, ST7735_YELLOW, ST7735_BLACK);

    snprintf (TX_Buffer, 64, "T:");
    ST7735_WriteString (36, 15 + i * 11, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

    snprintf (TX_Buffer, 64, "F:");
    ST7735_WriteString (111, 15 + i * 11, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);
  }
}
//*****************************************************************************

//*****************************************************************************
void ST7735_prepareMenu_5 (void)
{
  snprintf (TX_Buffer, 64, "TOOL:");
  ST7735_WriteString (14, 4, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "P: ");
  ST7735_WriteString (14, 15, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "I: ");
  ST7735_WriteString (14, 26, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "D: ");
  ST7735_WriteString (14, 37, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "ADC ($   'C):");
  ST7735_WriteString (14, 48, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "ADC ($   'C):");
  ST7735_WriteString (14, 59, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "ADC ($   'C):");
  ST7735_WriteString (14, 70, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "ADC ($   'C):");
  ST7735_WriteString (14, 81, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);
}
//*****************************************************************************

//*****************************************************************************
void ST7735_showMenu_0 (void)
{
  snprintf (TX_Buffer, 64, "%06d", VERSION);
  ST7735_WriteString (105, 102, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  if (flag_EEPROM_IRON_SETTINGS_OK)
  {
    snprintf (TX_Buffer, 64, " OK");
    ST7735_WriteString (112, 14, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);
  }
  else
  {
    snprintf (TX_Buffer, 64, " FAIL");
    ST7735_WriteString (112, 14, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);
  }
  HAL_Delay (100);

  if (flag_EEPROM_HOTAIR_SETTINGS_OK)
  {
    snprintf (TX_Buffer, 64, " OK");
    ST7735_WriteString (112, 25, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);
  }
  else
  {
    snprintf (TX_Buffer, 64, " FAIL");
    ST7735_WriteString (112, 25, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);
  }
  HAL_Delay (100);

  if (flag_EEPROM_HW_SETTINGS_OK)
  {
    snprintf (TX_Buffer, 64, " OK");
    ST7735_WriteString (112, 36, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);
  }
  else
  {
    snprintf (TX_Buffer, 64, " FAIL");
    ST7735_WriteString (112, 36, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);
  }
  HAL_Delay (100);

  if (flag_EEPROM_IRON_TOOLS_OK)
  {
    snprintf (TX_Buffer, 64, " OK");
    ST7735_WriteString (112, 47, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);
  }
  else
  {
    snprintf (TX_Buffer, 64, " FAIL");
    ST7735_WriteString (112, 47, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);
  }
  HAL_Delay (100);

  if (flag_EEPROM_HOTAIR_TOOLS_OK)
  {
    snprintf (TX_Buffer, 64, " OK");
    ST7735_WriteString (112, 58, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);
  }
  else
  {
    snprintf (TX_Buffer, 64, " FAIL");
    ST7735_WriteString (112, 58, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);
  }
  HAL_Delay (100);

  if (flag_EEPROM_IRON_PRESETS_OK)
  {
    snprintf (TX_Buffer, 64, " OK");
    ST7735_WriteString (112, 69, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);
  }
  else
  {
    snprintf (TX_Buffer, 64, " FAIL");
    ST7735_WriteString (112, 69, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);
  }
  HAL_Delay (100);

  if (flag_EEPROM_HOTAIR_PRESETS_OK)
  {
    snprintf (TX_Buffer, 64, " OK");
    ST7735_WriteString (112, 80, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);
  }
  else
  {
    snprintf (TX_Buffer, 64, " FAIL");
    ST7735_WriteString (112, 80, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);
  }

  // *** Loading parameters ***********
  IRON_set_temp = IRON_presets[0].temp;
  IRON_blow = IRON_presets[0].flow;

  HOTAIR_set_temp = HOTAIR_presets[0].temp;
  HOTAIR_blow = HOTAIR_presets[0].flow;
  // **********************************

  if ((HAL_GetTick () - time_stamp) > LOADING_SCREEN_TIME)
  {
    flag_changeMenu = true;
    menu = 1;
  }
}
//*****************************************************************************

//*****************************************************************************
void ST7735_showMenu_1 (void)
{
  if (!flag_change_IRON_preset)
  {
    if (flag_IRON_select_fun)
    {
      encoder_uploader (&htim3, &flag_reload_TIM3, &IRON_blow, IRON.min_flow, IRON.max_flow, IRON.step_flow);
    }
    else
    {
      encoder_uploader (&htim3, &flag_reload_TIM3, &IRON_set_temp, IRON.min_temp, IRON.max_temp, IRON.step_temp);
    }
  }
  else
  {
    IRON_set_temp = IRON_presets[IRON_preset_number].temp;
    IRON_blow = IRON_presets[IRON_preset_number].flow;
    flag_change_IRON_preset = false;
    flag_reload_TIM3 = true;
  }

  if (!flag_change_HOTAIR_preset)
  {
    if (flag_HOTAIR_select_fun)
    {
      encoder_uploader (&htim4, &flag_reload_TIM4, &HOTAIR_blow, HOTAIR.min_flow, HOTAIR.max_flow, HOTAIR.step_flow);
    }
    else
    {
      encoder_uploader (&htim4, &flag_reload_TIM4, &HOTAIR_set_temp, HOTAIR.min_temp, HOTAIR.max_temp, HOTAIR.step_temp);
    }
  }
  else
  {
    HOTAIR_set_temp = HOTAIR_presets[HOTAIR_preset_number].temp;
    HOTAIR_blow = HOTAIR_presets[HOTAIR_preset_number].flow;
    flag_change_HOTAIR_preset = false;
    flag_reload_TIM4 = true;
  }

  snprintf (TX_Buffer, 64, "%3i", IRON_set_temp);
  ST7735_WriteString (23, 2, TX_Buffer, Font_11x18, ST7735_YELLOW, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "%3i", IRON_blow);
  ST7735_WriteString (34, 20, TX_Buffer, Font_11x18, ST7735_CYAN, ST7735_BLACK);

  uint16_t IRON_color;
  if (flag_IRON_on)
  {
    IRON_color = 0xF800;
  }
  else
  {
    if (!IRON_ext_sensor)
    {
      IRON_color = 0x07FF;
    }
    else
    {
      IRON_color = 0x5555;
    }
  }

  if (flag_IRON_no_tool)
    snprintf (TX_Buffer, 64, "---");
  else
    snprintf (TX_Buffer, 64, "%3i", (uint16_t)(round(IRON_real_temp)));

  ST7735_WriteString (10, 48, TX_Buffer, Font_16x26, IRON_color, ST7735_BLACK);

  ST7735_showBar (13, 82, 60, 7, (float) (TIM5->CCR4) / (TIM5->ARR));

  snprintf (TX_Buffer, 64, "%3i", HOTAIR_set_temp);
  ST7735_WriteString (103, 2, TX_Buffer, Font_11x18, ST7735_YELLOW, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "%3i", HOTAIR_blow);
  ST7735_WriteString (114, 20, TX_Buffer, Font_11x18, ST7735_CYAN, ST7735_BLACK);

  uint16_t HOTAIR_color;
  if (flag_HOTAIR_on)
  {
    HOTAIR_color = 0xF800;
  }
  else
  {
    if (!flag_HOTAIR_NTC)
    {
      HOTAIR_color = 0x07FF;
    }
    else
    {
      HOTAIR_color = 0x5555;
    }
  }

  if (flag_HOTAIR_no_tool)
    snprintf (TX_Buffer, 64, "---");
  else
    snprintf (TX_Buffer, 64, "%3i", (uint16_t)HOTAIR_real_temp);
  ST7735_WriteString (90, 48, TX_Buffer, Font_16x26, HOTAIR_color, ST7735_BLACK);

  ST7735_showBar (87, 82, 60, 7, (float) (TIM2->CCR1) / (TIM2->ARR));

  snprintf (TX_Buffer, 64, "%1i", IRON_preset_number + 1);
  ST7735_WriteString (3, 81, TX_Buffer, Font_7x10, ST7735_YELLOW, ST7735_COLOR565(100, 100, 65));

  snprintf (TX_Buffer, 64, "%1i", IRON_tool_number + 1);
  ST7735_WriteString (3, 94, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_COLOR565(100, 100, 65));

  snprintf (TX_Buffer, 64, "%1i", HOTAIR_preset_number + 1);
  ST7735_WriteString (150, 81, TX_Buffer, Font_7x10, ST7735_YELLOW, ST7735_COLOR565(100, 100, 65));

  uint8_t fract_part = 0;
  if (flag_IRON_off)
  {
    snprintf (TX_Buffer, 64, "-----");
  }
  else if (flag_IRON_hold)
  {
    snprintf (TX_Buffer, 64, " HOLD");
  }
  else
  {
    fract_part = round ((IRON_amperage - (uint16_t) IRON_amperage) * 10);
    if (fract_part == 10)
    {
      IRON_amperage += 1;
      fract_part = 0;
    }
    snprintf (TX_Buffer, 64, "%2i.%1iA", (uint16_t) IRON_amperage, fract_part);
  }

  ST7735_WriteString (15, 94, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  fract_part = round ((voltage24 - (uint16_t) voltage24) * 10);
  if (fract_part == 10)
  {
    voltage24 += 1;
    fract_part = 0;
  }
  snprintf (TX_Buffer, 64, "%02i.%1iV", (uint16_t) voltage24, fract_part);
  ST7735_WriteString (65, 94, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  if (flag_HOTAIR_off)
  {
    snprintf (TX_Buffer, 64, "-----");
  }
  else if (flag_HOTAIR_hold)
  {
    snprintf (TX_Buffer, 64, " HOLD");
  }
  else
  {
    snprintf (TX_Buffer, 64, "     ");
  }

  ST7735_WriteString (108, 94, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);
}
//*****************************************************************************

//*****************************************************************************
void ST7735_showMenu_2_p1 (void)
{
  switch (selected_sub_menu_settings)
  {
    case (0) :
      snprintf (TX_Buffer, 64, "HOTAIR SETTINGS:");
      ST7735_WriteString (14, 3, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);
      ST7735_showMenu_2_p2 (&HOTAIR);
      break;
    case (1) :
      snprintf (TX_Buffer, 64, "  IRON SETTINGS:");
      ST7735_WriteString (14, 3, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);
      ST7735_showMenu_2_p2 (&IRON);
      break;
    case (2) :
      snprintf (TX_Buffer, 64, "    HW SETTINGS:");
      ST7735_WriteString (14, 3, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);
      ST7735_showMenu_2_p3();
      break;
  }
}
//*****************************************************************************

//*****************************************************************************
void ST7735_showMenu_2_p2 (settings_t *set)
{
  snprintf (TX_Buffer, 64, "MIN TEMP:     %3i'C", set->min_temp);
  ST7735_WriteString (14, 15, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "MAX TEMP:     %3i'C", set->max_temp);
  ST7735_WriteString (14, 26, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "STEP TEMP:    %3i'C", set->step_temp);
  ST7735_WriteString (14, 37, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "MIN FLOW:     %3i %%", set->min_flow);
  ST7735_WriteString (14, 48, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "MAX FLOW:     %3i %%", set->max_flow);
  ST7735_WriteString (14, 59, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "STEP FLOW:    %3i %%", set->step_flow);
  ST7735_WriteString (14, 70, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  switch (set->ext_NTC)
  {
    case (0):
      snprintf (TX_Buffer, 64, "TOOL NTC:       N  ");
      break;
    case (1):
      snprintf (TX_Buffer, 64, "TOOL NTC:    MF52  ");
      break;
    case (2):
      snprintf (TX_Buffer, 64, "TOOL NTC:  CB3435  ");
      break;
    case (3):
      snprintf (TX_Buffer, 64, "TOOL NTC:  CB3950  ");
      break;
    case (4):
      snprintf (TX_Buffer, 64, "TOOL NTC:  CB3988  ");
      break;
  }
  ST7735_WriteString (14, 81, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);


  if (set->sensor)
    snprintf (TX_Buffer, 64, "TOOL SENSOR:    Y  ");
  else
    snprintf (TX_Buffer, 64, "TOOL SENSOR:    N  ");
  ST7735_WriteString (14, 92, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "SLEEP TIMER:  %3i S", set->sleep_timer);
  ST7735_WriteString (14, 103, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  for (uint8_t i = 0; i < 10; i++)
  {
    if (i == selected_item)
      snprintf (TX_Buffer, 64, ">");
    else
      snprintf (TX_Buffer, 64, " ");

    ST7735_WriteString (3, 4 + i * 11, TX_Buffer, Font_7x10, ST7735_CYAN, ST7735_BLACK);
  }

  switch (selected_item)
  {
    case 0:
      encoder_uploader (&htim4, &flag_reload_TIM4, &selected_sub_menu_settings, 0, 2, 1);
      break;
    case 1:
      encoder_uploader (&htim4, &flag_reload_TIM4, &set->min_temp, LIM_MIN_TEMP, LIM_MAX_TEMP,
      LIM_MIN_STEP_TEMP);
      break;
    case 2:
      encoder_uploader (&htim4, &flag_reload_TIM4, &set->max_temp, LIM_MIN_TEMP, LIM_MAX_TEMP,
      LIM_MIN_STEP_TEMP);
      break;
    case 3:
      encoder_uploader (&htim4, &flag_reload_TIM4, &set->step_temp, LIM_MIN_STEP_TEMP,
      LIM_MAX_STEP_TEMP,
                        LIM_MIN_STEP_TEMP);
      break;
    case 4:
      encoder_uploader (&htim4, &flag_reload_TIM4, &set->min_flow, LIM_MIN_FAN, LIM_MAX_FAN,
      LIM_MIN_STEP_FAN);
      break;
    case 5:
      encoder_uploader (&htim4, &flag_reload_TIM4, &set->max_flow, LIM_MIN_FAN, LIM_MAX_FAN,
      LIM_MIN_STEP_FAN);
      break;
    case 6:
      encoder_uploader (&htim4, &flag_reload_TIM4, &set->step_flow, LIM_MIN_STEP_FAN,
      LIM_MAX_STEP_FAN,
                        1);
      break;
    case 7:
      encoder_uploader (&htim4, &flag_reload_TIM4, &set->ext_NTC, 0, 4, 1);
      break;
    case 8:
      encoder_uploader (&htim4, &flag_reload_TIM4, &set->sensor, 0, 1, 1);
      break;
    case 9:
      encoder_uploader (&htim4, &flag_reload_TIM4, &set->sleep_timer, LIM_MIN_SLEEP_TIMER,
      LIM_MAX_SLEEP_TIMER,
                        LIM_STEP_SLEEP_TIMER);
      break;
  }
}
//*****************************************************************************

//*****************************************************************************
void ST7735_showMenu_2_p3 (void)
{
  switch (HW_SET.NTC)
  {
    case (0):
      snprintf (TX_Buffer, 64, "INT NTC:        N  ");
      break;
    case (1):
      snprintf (TX_Buffer, 64, "INT NTC:     MF52  ");
      break;
    case (2):
      snprintf (TX_Buffer, 64, "INT NTC:   CB3435  ");
      break;
    case (3):
      snprintf (TX_Buffer, 64, "INT NTC:   CB3950  ");
      break;
    case (4):
      snprintf (TX_Buffer, 64, "INT NTC:   CB3988  ");
      break;
  }
  ST7735_WriteString (14, 15, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "0 current:   %4i  ", HW_SET.ZERO_CURRENT);
  ST7735_WriteString (14, 26, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "K current:    %3i  ", HW_SET.K_CURRENT);
  ST7735_WriteString (14, 37, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "K voltage:    %3i  ", HW_SET.K_VOLTAGE);
  ST7735_WriteString (14, 48, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "K INT NTC:    %3i  ", HW_SET.K_INT_NTC);
  ST7735_WriteString (14, 59, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "K IRON NTC:   %3i  ", HW_SET.K_IRON_NTC);
  ST7735_WriteString (14, 70, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "K HOTAIR NTC: %3i  ", HW_SET.K_HOTAIR_NTC);
  ST7735_WriteString (14, 81, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "TIME OFF:    %4i S", HW_SET.time_off);
  ST7735_WriteString (14, 92, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "OVERCURRENT:   %2i A", HW_SET.overcurrent_protection);
  ST7735_WriteString (14, 103, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  for (uint8_t i = 0; i < 10; i++)
  {
    if (i == selected_item)
      snprintf (TX_Buffer, 64, ">");
    else
      snprintf (TX_Buffer, 64, " ");

    ST7735_WriteString (3, 4 + i * 11, TX_Buffer, Font_7x10, ST7735_CYAN, ST7735_BLACK);
  }

  switch (selected_item)
  {
    case 0:
      encoder_uploader (&htim4, &flag_reload_TIM4, &selected_sub_menu_settings, 0, 2, 1);
      break;
    case 1:
      encoder_uploader (&htim4, &flag_reload_TIM4, &HW_SET.NTC, 0, 4, 1);
      break;
    case 2:
      encoder_uploader (&htim4, &flag_reload_TIM4, &HW_SET.ZERO_CURRENT, 2081, 2481, 1);
      break;
    case 3:
      encoder_uploader (&htim4, &flag_reload_TIM4, &HW_SET.K_CURRENT, 0, 200, 1);
      break;
    case 4:
      encoder_uploader (&htim4, &flag_reload_TIM4, &HW_SET.K_VOLTAGE, 0, 200, 1);
      break;
    case 5:
      encoder_uploader (&htim4, &flag_reload_TIM4, &HW_SET.K_INT_NTC, 0, 200, 1);
      break;
    case 6:
      encoder_uploader (&htim4, &flag_reload_TIM4, &HW_SET.K_IRON_NTC, 0, 200, 1);
      break;
    case 7:
      encoder_uploader (&htim4, &flag_reload_TIM4, &HW_SET.K_HOTAIR_NTC, 0, 200, 1);
      break;
    case 8:
      encoder_uploader (&htim4, &flag_reload_TIM4, &HW_SET.time_off, 0, 30*60, 30);
      break;
    case 9:
      encoder_uploader (&htim4, &flag_reload_TIM4, &HW_SET.overcurrent_protection, 0, 10, 1);
      break;
  }
}
//*****************************************************************************

//*****************************************************************************
void ST7735_showMenu_3 (preset_t *preset, uint8_t max_presets, settings_t *set)
{
  for (uint8_t i = 0; i < max_presets; i++)
  {
    snprintf (TX_Buffer, 64, "%3i'C", preset[i].temp);
    ST7735_WriteString (50, 15 + i * 11, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

    snprintf (TX_Buffer, 64, "%3i%%", preset[i].flow);
    ST7735_WriteString (125, 15 + i * 11, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);
  }

  if (selected_item == 255)
    selected_item = max_presets * 2 - 1;
  else if (selected_item >= max_presets * 2)
    selected_item = 0;

  for (uint8_t i = 0; i < max_presets * 2; i++)
  {
    if (i == selected_item)
      snprintf (TX_Buffer, 64, ">");
    else
      snprintf (TX_Buffer, 64, " ");

    if (i % 2 == 0)
      ST7735_WriteString (29, 15 + (i / 2) * 11, TX_Buffer, Font_7x10, ST7735_CYAN,
      ST7735_BLACK);
    else
      ST7735_WriteString (104, 15 + (i / 2) * 11, TX_Buffer, Font_7x10, ST7735_CYAN,
      ST7735_BLACK);
  }

  switch (selected_item % 2)
  {
    case 0:
      encoder_uploader (&htim4, &flag_reload_TIM4, &preset[selected_item / 2].temp, set->min_temp, set->max_temp, set->step_temp);
      break;
    case 1:
      encoder_uploader (&htim4, &flag_reload_TIM4, &preset[selected_item / 2].flow, set->min_flow, set->max_flow, set->step_flow);
      break;
  }
}
//*****************************************************************************

//*****************************************************************************
void ST7735_showMenu_5_p1 (void)
{
  if (selected_tool == 0)
  {
    snprintf (TX_Buffer, 64, "HOTAIR");
  }
  else
  {
    snprintf (TX_Buffer, 64, "IRON#%1i", selected_tool);
  }
  ST7735_WriteString (56, 4, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  if (selected_item == 255)
    selected_item = 7;
  else if (selected_item >= 8)
    selected_item = 0;

  for (uint8_t i = 0; i < 8; i++)
  {
    if (i == selected_item)
      snprintf (TX_Buffer, 64, ">");
    else
      snprintf (TX_Buffer, 64, " ");

    ST7735_WriteString (3, 4 + i * 11, TX_Buffer, Font_7x10, ST7735_CYAN, ST7735_BLACK);
  }

  if (!selected_item)
    encoder_uploader (&htim4, &flag_reload_TIM4, &selected_tool, 0, IRON_TOOLS, 1);

  encoder_uploader (&htim3, &flag_reload_TIM3, &calibration_correction, 0, LIM_CALIBRATION_RANGEWIDTH, 1);

  if (selected_tool)
    ST7735_showMenu_5_p2 (&IRON_tools[selected_tool - 1]);
  else
    ST7735_showMenu_5_p2 (&HOTAIR_tools);
}
//*****************************************************************************

//*****************************************************************************
void ST7735_showMenu_5_p2 (tools_t *tool)
{
  snprintf (TX_Buffer, 64, "%3i", tool->P);
  ST7735_WriteString (125, 15, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "%3i", tool->I);
  ST7735_WriteString (125, 26, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  snprintf (TX_Buffer, 64, "%3i", tool->D);
  ST7735_WriteString (125, 37, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

  tool->calibration_table[4].temp_value = approximateT (tool->calibration_table[4].raw_value, tool->calibration_table[3].raw_value,
                                                        tool->calibration_table[2].raw_value, tool->calibration_table[3].temp_value,
                                                        tool->calibration_table[2].temp_value);

  for (uint8_t i = 0; i < 4; i++)
  {
    snprintf (TX_Buffer, 64, "%3i", tool->calibration_table[i].temp_value);
    ST7735_WriteString (56, 48 + i * 11, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

    snprintf (TX_Buffer, 64, "%4i", tool->calibration_table[i].raw_value);
    ST7735_WriteString (118, 48 + i * 11, TX_Buffer, Font_7x10, ST7735_YELLOW, ST7735_BLACK);
  }

  if (selected_tool)
  {
    IRON_tool_number = selected_tool - 1;
    if (selected_item > 4)
    {
      IRON_set_temp = tool->calibration_table[selected_item - 4].temp_value + calibration_correction - LIM_CALIBRATION_RANGEWIDTH / 2;
      flag_IRON_on = true;
    }
    else
    {
      IRON_set_temp = 0;
    }

    if (IRON.ext_NTC && flag_IRON_NTC)
    {
      IRON_set_temp += temperature_IRON_NTC;
      snprintf (TX_Buffer, 64, "%03i'C", (uint16_t)temperature_IRON_NTC);
    }
    else
    {
      IRON_set_temp += temperature_ONBOARD_NTC;
      snprintf (TX_Buffer, 64, "%03i'C", (uint16_t)temperature_ONBOARD_NTC);
    }
    ST7735_WriteString (14, 92, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

    if (!flag_IRON_no_tool)
    {
      snprintf (TX_Buffer, 64, "$%03i'C", (uint16_t)temperature_IRON_TC);
      ST7735_WriteString (56, 92, TX_Buffer, Font_7x10, ST7735_RED, ST7735_BLACK);
    }

    snprintf (TX_Buffer, 64, "%04i", adc_filtered[0]);
    ST7735_WriteString (119, 92, TX_Buffer, Font_7x10, ST7735_YELLOW, ST7735_BLACK);
  }
  else
  {
    if (selected_item > 4)
    {
      HOTAIR_set_temp = tool->calibration_table[selected_item - 4].temp_value + calibration_correction - LIM_CALIBRATION_RANGEWIDTH / 2;
    }
    else
    {
      HOTAIR_set_temp = 0;
    }

    if (HOTAIR.ext_NTC && flag_HOTAIR_NTC)
    {
      HOTAIR_set_temp += temperature_HOTAIR_NTC;
      snprintf (TX_Buffer, 64, "%03i'C", (uint16_t)temperature_HOTAIR_NTC);
    }
    else
    {
      HOTAIR_set_temp += temperature_ONBOARD_NTC;
      snprintf (TX_Buffer, 64, "%03i'C", (uint16_t)temperature_ONBOARD_NTC);
    }
    ST7735_WriteString (14, 92, TX_Buffer, Font_7x10, ST7735_GREEN, ST7735_BLACK);

    if (!flag_HOTAIR_no_tool)
    {
      snprintf (TX_Buffer, 64, "$%03i'C", (uint16_t)temperature_HOTAIR_TC);
      ST7735_WriteString (56, 92, TX_Buffer, Font_7x10, ST7735_RED, ST7735_BLACK);
    }

    snprintf (TX_Buffer, 64, "%04i", adc_filtered[1]);
    ST7735_WriteString (119, 92, TX_Buffer, Font_7x10, ST7735_YELLOW, ST7735_BLACK);
  }

  ST7735_showBar (10, 105, 140, 4, ((float) calibration_correction) / LIM_CALIBRATION_RANGEWIDTH);

  switch (selected_item)
  {
    case 1:
      encoder_uploader (&htim4, &flag_reload_TIM4, &tool->P, LIM_MIN_PID_K, LIM_MAX_PID_K, LIM_STEP_PID_K);
      break;
    case 2:
      encoder_uploader (&htim4, &flag_reload_TIM4, &tool->I, LIM_MIN_PID_K, LIM_MAX_PID_K, LIM_STEP_PID_K);
      break;
    case 3:
      encoder_uploader (&htim4, &flag_reload_TIM4, &tool->D, LIM_MIN_PID_K, LIM_MAX_PID_K, LIM_STEP_PID_K);
      break;
    case 4:
      encoder_uploader (&htim4, &flag_reload_TIM4, &tool->calibration_table[0].raw_value, LIM_MIN_ADC, LIM_MAX_ADC, LIM_STEP_ADC);
      break;
    case 5:
      encoder_uploader (&htim4, &flag_reload_TIM4, &tool->calibration_table[1].raw_value, LIM_MIN_ADC, LIM_MAX_ADC, LIM_STEP_ADC);
      break;
    case 6:
      encoder_uploader (&htim4, &flag_reload_TIM4, &tool->calibration_table[2].raw_value, LIM_MIN_ADC, LIM_MAX_ADC, LIM_STEP_ADC);
      break;
    case 7:
      encoder_uploader (&htim4, &flag_reload_TIM4, &tool->calibration_table[3].raw_value, LIM_MIN_ADC, LIM_MAX_ADC, LIM_STEP_ADC);
      break;
  }
}
//*****************************************************************************

//*****************************************************************************
void ST7735_showBar (uint16_t x, uint16_t y, uint16_t length, uint16_t width, float percent)
{
  for (uint16_t k = y; k < y + width; k++)
  {
    for (int j = x; j < x + length; j++)
    {
      if (j < x + (length * percent))
      {
        ST7735_DrawPixel (j, k, 0xF800);
      }
      else
      {
        ST7735_DrawPixel (j, k, 0x5555);
      }
    }
  }
}
//*****************************************************************************

//*****************************************************************************
bool EEPROM_Init (void)
{
  if (EE24_Init (&ee24, &hi2c1, EE24_ADDRESS_DEFAULT))
    return 1;
  else
    return 0;
}
//*****************************************************************************

//*****************************************************************************
void EEPROM_Show (void)
{
  uint8_t dataRead_Full_ee24[EEPROM_SIZE];
  char data_str[64] = { 0, };

  EE24_Read (&ee24, 0u, dataRead_Full_ee24, EEPROM_SIZE, 1000);
  HAL_Delay (10);

  snprintf (data_str, 63, "\n");
  HAL_UART_Transmit (UART_HEANDLER, (uint8_t*) data_str, strlen (data_str), 10);

  snprintf (data_str, 63, "EEPROM  INIT ");
  HAL_UART_Transmit (UART_HEANDLER, (uint8_t*) data_str, strlen (data_str), 10);

  for (uint8_t j = 0; j < 32; j++)
  {
    snprintf (data_str, 63, " %03d ", j);
    HAL_UART_Transmit (UART_HEANDLER, (uint8_t*) data_str, strlen (data_str), 10);
  }
  snprintf (data_str, 63, "\n");
  HAL_UART_Transmit (UART_HEANDLER, (uint8_t*) data_str, strlen (data_str), 10);

  snprintf (data_str, 63, "_____________");
  HAL_UART_Transmit (UART_HEANDLER, (uint8_t*) data_str, strlen (data_str), 10);

  for (uint8_t j = 0; j < 32; j++)
  {
    snprintf (data_str, 63, "_____");
    HAL_UART_Transmit (UART_HEANDLER, (uint8_t*) data_str, strlen (data_str), 10);
  }
  snprintf (data_str, 63, "\n");
  HAL_UART_Transmit (UART_HEANDLER, (uint8_t*) data_str, strlen (data_str), 10);

  for (uint8_t i = 0; i < EEPROM_SIZE / 32; i++)
  {
    snprintf (data_str, 63, "%04d-%04d I  ", 32 * i, 32 * i + 31);
    HAL_UART_Transmit (UART_HEANDLER, (uint8_t*) data_str, strlen (data_str), 10);

    for (uint8_t j = 0; j < 32; j++)
    {
      snprintf (data_str, 63, "0x%02X ", (uint8_t) dataRead_Full_ee24[32 * i + j]);
      HAL_UART_Transmit (UART_HEANDLER, (uint8_t*) data_str, strlen (data_str), 10);
    }
    snprintf (data_str, 63, "\n");
    HAL_UART_Transmit (UART_HEANDLER, (uint8_t*) data_str, strlen (data_str), 10);
  }
  snprintf (data_str, 63, "\n");
  HAL_UART_Transmit (UART_HEANDLER, (uint8_t*) data_str, strlen (data_str), 10);
}
//*****************************************************************************

//*****************************************************************************
void EEPROM_Read_All (void)
{
  if (EE24_ReadCircle (&ee24, EEPROM_IRON_SETTINGS_START, EEPROM_IRON_SETTINGS_STOP, CTRL_BYTE, &IRON, sizeof(IRON), 10))
  {
    flag_EEPROM_IRON_SETTINGS_OK = true;
  }
  else
  {
    IRON = DEF_IRON;
  }

  if (EE24_ReadCircle (&ee24, EEPROM_HOTAIR_SETTINGS_START, EEPROM_HOTAIR_SETTINGS_STOP, CTRL_BYTE, &HOTAIR, sizeof(HOTAIR), 10))
  {
    flag_EEPROM_HOTAIR_SETTINGS_OK = true;
  }
  else
  {
    HOTAIR = DEF_HOTAIR;
  }

  if (EE24_ReadCircle (&ee24, EEPROM_HW_SETTINGS_START, EEPROM_HW_SETTINGS_STOP, CTRL_BYTE, &HW_SET, sizeof(HW_SET), 10))
  {
    flag_EEPROM_HW_SETTINGS_OK = true;
  }
  else
  {
    HW_SET = DEF_HW_SET;
  }

  if (EE24_ReadCircle (&ee24, EEPROM_IRON_TOOLS_START, EEPROM_IRON_TOOLS_STOP, CTRL_BYTE, &IRON_tools, sizeof(IRON_tools), 10))
  {
    flag_EEPROM_IRON_TOOLS_OK = true;
  }
  else
  {
    for (uint8_t i = 0; i < IRON_TOOLS; i++)
    {
      IRON_tools[i].P = DEF_IRON_TOOL_P;
      IRON_tools[i].I = DEF_IRON_TOOL_I;
      IRON_tools[i].D = DEF_IRON_TOOL_D;
      for (uint8_t k = 0; k <= 5; k++)
      {
        IRON_tools[i].calibration_table[k] = DEF_CAL_TABLE_IRON[k];
      }
      /*
      IRON_tools[i].calibration_table[4].temp_value = approximateT ( IRON_tools[i].calibration_table[4].raw_value,  IRON_tools[i].calibration_table[3].raw_value,
                                                                     IRON_tools[i].calibration_table[2].raw_value,  IRON_tools[i].calibration_table[3].temp_value,
                                                                     IRON_tools[i].calibration_table[2].temp_value);
                                                                     */
    }
  }

  if (EE24_ReadCircle (&ee24, EEPROM_HOTAIR_TOOLS_START, EEPROM_HOTAIR_TOOLS_STOP, CTRL_BYTE, &HOTAIR_tools, sizeof(HOTAIR_tools), 10))
  {
    flag_EEPROM_HOTAIR_TOOLS_OK = true;
  }
  else
  {
    HOTAIR_tools.P = DEF_HOTAIR_TOOL_P;
    HOTAIR_tools.I = DEF_HOTAIR_TOOL_I;
    HOTAIR_tools.D = DEF_HOTAIR_TOOL_D;
    for (uint8_t k = 0; k < 5; k++)
    {
      HOTAIR_tools.calibration_table[k] = DEF_CAL_TABLE_HOTAIR[k];
    }
    /*
    HOTAIR_tools.calibration_table[4].temp_value = approximateT ( HOTAIR_tools.calibration_table[4].raw_value,  HOTAIR_tools.calibration_table[3].raw_value,
                                                                  HOTAIR_tools.calibration_table[2].raw_value,  HOTAIR_tools.calibration_table[3].temp_value,
                                                                  HOTAIR_tools.calibration_table[2].temp_value);
                                                                  */

  }

  if (EE24_ReadCircle (&ee24, EEPROM_IRON_PRESETS_START, EEPROM_IRON_PRESETS_STOP, CTRL_BYTE, &IRON_presets, sizeof(IRON_presets), 10))
  {
    flag_EEPROM_IRON_PRESETS_OK = true;
  }
  else
  {
    for (uint8_t i = 0; i < IRON_PRESETS; i++)
    {
      IRON_presets[i] = DEF_IRON_presets[i];
    }
  }

  if (EE24_ReadCircle (&ee24, EEPROM_HOTAIR_PRESETS_START, EEPROM_HOTAIR_PRESETS_STOP, CTRL_BYTE, &HOTAIR_presets, sizeof(HOTAIR_presets),
                       10))
  {
    flag_EEPROM_HOTAIR_PRESETS_OK = true;
  }
  else
  {
    for (uint8_t i = 0; i < HOTAIR_PRESETS; i++)
    {
      HOTAIR_presets[i] = DEF_HOTAIR_presets[i];
    }
  }
}
//*****************************************************************************

//*****************************************************************************
void EEPROM_Write_All (void)
{
  if (flag_write_IRON_settings)
  {
    EE24_WriteCircle (&ee24, EEPROM_IRON_SETTINGS_START, EEPROM_IRON_SETTINGS_STOP, CTRL_BYTE, &IRON, sizeof(IRON), 10);
    flag_write_IRON_settings = false;
  }

  if (flag_write_HOTAIR_settings)
  {
    EE24_WriteCircle (&ee24, EEPROM_HOTAIR_SETTINGS_START, EEPROM_HOTAIR_SETTINGS_STOP, CTRL_BYTE, &HOTAIR, sizeof(HOTAIR), 10);
    flag_write_HOTAIR_settings = false;
  }

  if (flag_write_HW_settings)
  {
    EE24_WriteCircle (&ee24, EEPROM_HW_SETTINGS_START, EEPROM_HW_SETTINGS_STOP, CTRL_BYTE, &HW_SET, sizeof(HW_SET), 10);
    flag_write_HW_settings = false;
  }

  if (flag_write_IRON_tools)
  {
    EE24_WriteCircle (&ee24, EEPROM_IRON_TOOLS_START, EEPROM_IRON_TOOLS_STOP, CTRL_BYTE, &IRON_tools, sizeof(IRON_tools), 10);
    flag_write_IRON_tools = false;
  }

  if (flag_write_HOTAIR_tools)
  {
    EE24_WriteCircle (&ee24, EEPROM_HOTAIR_TOOLS_START, EEPROM_HOTAIR_TOOLS_STOP, CTRL_BYTE, &HOTAIR_tools, sizeof(HOTAIR_tools), 10);
    flag_write_HOTAIR_tools = false;
  }

  if (flag_write_IRON_presets)
  {
    EE24_WriteCircle (&ee24, EEPROM_IRON_PRESETS_START, EEPROM_IRON_PRESETS_STOP, CTRL_BYTE, &IRON_presets, sizeof(IRON_presets), 10);
    flag_write_IRON_presets = false;
  }

  if (flag_write_HOTAIR_presets)
  {
    EE24_WriteCircle (&ee24, EEPROM_HOTAIR_PRESETS_START, EEPROM_HOTAIR_PRESETS_STOP, CTRL_BYTE, &HOTAIR_presets, sizeof(HOTAIR_presets),
                      10);
    flag_write_HOTAIR_presets = false;
  }
}
//*****************************************************************************

//*****************************************************************************
void EEPROM_RESET (void)
{
  uint8_t empty_byte = 0xFF;

  for (uint16_t i = 0; i < EEPROM_SIZE; i++)
  {
    EE24_Write (&ee24, i, empty_byte, 1u, 100);
  }
}
//*****************************************************************************

//*****************************************************************************
void Button_React (uint16_t key_number, bool longpress)
{
  switch (menu)
  {
    //*********************************************************************************************
    case (0):                                                                  // Loading
      switch (key_number)
      {

        case (K_1_Pin):
          if (longpress)
          {

          }
          else
          {
            menu = 1;
            flag_changeMenu = true;
            flag_reload_TIM3 = true;
            flag_reload_TIM4 = true;
          }
          break;

        case (K_2_Pin):
          if (longpress)
          {
            EEPROM_RESET ();
            HAL_NVIC_SystemReset ();
          }
          else
          {

          }
          break;

        case (K_3_Pin):
          if (longpress)
          {

          }
          else
          {

          }
          break;

        case (K_4_Pin):
          if (longpress)
          {

          }
          else
          {

          }
          break;

      }
      break;
    //*********************************************************************************************
    //*********************************************************************************************
    case (1):                                                                  // Main menu
      switch (key_number)
      {

        case (K_1_Pin):
          if (longpress)
          {
            menu = 4;
            flag_changeMenu = true;
            flag_reload_TIM4 = true;
          }
          else
          {
            flag_HOTAIR_WAKEUP = true;
            flag_change_HOTAIR_preset = true;
            if (HOTAIR_preset_number >= HOTAIR_PRESETS - 1)
              HOTAIR_preset_number = 0;
            else
              HOTAIR_preset_number++;
          }
          break;

        case (K_2_Pin):
          if (longpress)
          {
            // OFF
            HAL_GPIO_WritePin (PS_ON_GPIO_Port, PS_ON_Pin, 0);
            HAL_GPIO_WritePin (SSR_ON_GPIO_Port, SSR_ON_Pin, 0);
          }
          else
          {
            menu = 2;
            flag_changeMenu = true;
            flag_reload_TIM4 = true;
          }
          break;

        case (K_3_Pin):
          if (longpress)
          {
            menu = 5;
            flag_changeMenu = true;
            flag_reload_TIM3 = true;
            flag_reload_TIM4 = true;
            calibration_correction = LIM_CALIBRATION_RANGEWIDTH / 2;
            IRON_set_temp_RR = IRON_set_temp;
            IRON_set_temp = 0;
            HOTAIR_set_temp_RR = HOTAIR_set_temp;
            HOTAIR_set_temp = 0;
          }
          else
          {
            flag_change_IRON_tool = true;
            if (IRON_tool_number >= IRON_TOOLS - 1)
              IRON_tool_number = 0;
            else
              IRON_tool_number++;
          }
          break;

        case (K_4_Pin):
          if (longpress)
          {
            menu = 3;
            flag_changeMenu = true;
            flag_reload_TIM4 = true;
          }
          else
          {
            flag_IRON_WAKEUP = true;
            flag_change_IRON_preset = true;
            if (IRON_preset_number >= IRON_PRESETS - 1)
              IRON_preset_number = 0;
            else
              IRON_preset_number++;
          }
          break;

        case (EN1_button_Pin):
          flag_IRON_WAKEUP = true;
          if (longpress)
          {
            if (!flag_IRON_no_tool)
            {
              if (flag_IRON_on)
              {
                flag_IRON_on = false;
                TIM5->CCR4 = 0;
              }
              else
              {
                flag_IRON_on = true;
                flag_IRON_preheat_start = true;
              }
            }
          }
          else
          {
            flag_IRON_select_fun = !flag_IRON_select_fun;
            flag_reload_TIM3 = true;
          }
          break;

        case (EN2_button_Pin):
          flag_HOTAIR_WAKEUP = true;
          if (longpress)
          {
            if (!flag_HOTAIR_no_tool)
            {
              if (flag_HOTAIR_on)
              {
                flag_HOTAIR_on = false;
              }
              else
              {
                flag_HOTAIR_on = true;
              }
            }
          }
          else
          {
            flag_HOTAIR_select_fun = !flag_HOTAIR_select_fun;
            flag_reload_TIM4 = true;
          }
          break;

      }
      break;
    //*********************************************************************************************

    //*********************************************************************************************
    case (2):                                                              // IRON settings
      switch (key_number)
      {

        case (K_1_Pin):
          if (longpress)
          {

          }
          else
          {
            menu = 1;
            selected_item = 0;
            flag_changeMenu = true;
            flag_reload_TIM4 = true;
          }
          break;

        case (K_2_Pin):
          if (longpress)
          {

          }
          else
          {
            switch (selected_sub_menu_settings)
            {
              case (0):
                flag_write_HOTAIR_settings = true;
                break;
              case (1):
                flag_write_IRON_settings = true;
                break;
              case (2):
                flag_write_HW_settings = true;
                break;
            }
          }
          break;

        case (K_3_Pin):
          if (longpress)
          {

          }
          else
          {
            flag_reload_TIM4 = true;
            if (selected_item < 9)
              selected_item++;
            else
              selected_item = 0;
          }
          break;

        case (K_4_Pin):
          if (longpress)
          {

          }
          else
          {
            flag_reload_TIM4 = true;
            if (selected_item > 0)
              selected_item--;
            else
              selected_item = 9;
          }
          break;

        case (EN1_button_Pin):
          if (longpress)
          {

          }
          else
          {

          }
          break;

        case (EN2_button_Pin):
          if (longpress)
          {

          }
          else
          {

          }
          break;

      }
      break;
    //*********************************************************************************************

    //*********************************************************************************************
    case (3):                                                                  // IRON presets
      switch (key_number)
      {

        case (K_1_Pin):
          if (longpress)
          {

          }
          else
          {
            menu = 1;
            selected_item = 0;
            flag_changeMenu = true;
            flag_reload_TIM4 = true;
          }
          break;

        case (K_2_Pin):
          if (longpress)
          {

          }
          else
          {
            flag_write_IRON_presets = true;
          }
          break;

        case (K_3_Pin):
          if (longpress)
          {

          }
          else
          {
            flag_reload_TIM4 = true;
            selected_item++;
          }
          break;

        case (K_4_Pin):
          if (longpress)
          {

          }
          else
          {
            flag_reload_TIM4 = true;
            selected_item--;
          }
          break;

        case (EN1_button_Pin):
          if (longpress)
          {

          }
          else
          {

          }
          break;

        case (EN2_button_Pin):
          if (longpress)
          {

          }
          else
          {

          }
          break;

      }
      break;
    //*********************************************************************************************

    //*********************************************************************************************
    case (4):                                                                  // HOTAIR presets
      switch (key_number)
      {

        case (K_1_Pin):
          if (longpress)
          {

          }
          else
          {
            menu = 1;
            selected_item = 0;
            flag_changeMenu = true;
            flag_reload_TIM4 = true;
          }
          break;

        case (K_2_Pin):
          if (longpress)
          {

          }
          else
          {
            flag_write_HOTAIR_presets = true;
          }
          break;

        case (K_3_Pin):
          if (longpress)
          {

          }
          else
          {
            flag_reload_TIM4 = true;
            selected_item++;
          }
          break;

        case (K_4_Pin):
          if (longpress)
          {

          }
          else
          {
            flag_reload_TIM4 = true;
            selected_item--;
          }
          break;

        case (EN1_button_Pin):
          if (longpress)
          {

          }
          else
          {

          }
          break;

        case (EN2_button_Pin):
          if (longpress)
          {

          }
          else
          {

          }
          break;

      }
      break;
    //*********************************************************************************************

    //*********************************************************************************************
    case (5):                                                                  // TOOLS
      switch (key_number)
      {

        case (K_1_Pin):
          if (longpress)
          {

          }
          else
          {
            menu = 1;
            selected_item = 0;
            flag_changeMenu = true;
            flag_reload_TIM3 = true;
            flag_reload_TIM4 = true;

            IRON_set_temp = IRON_set_temp_RR;
            HOTAIR_set_temp = HOTAIR_set_temp_RR;

          }
          break;

        case (K_2_Pin):
          if (longpress)
          {

          }
          else
          {
            flag_write_IRON_tools = true;
            flag_write_HOTAIR_tools = true;
            flag_change_IRON_tool = true;
            flag_change_HOTAIR_tool = true;
          }
          break;

        case (K_3_Pin):
          if (longpress)
          {

          }
          else
          {
            calibration_correction = LIM_CALIBRATION_RANGEWIDTH / 2;
            flag_reload_TIM3 = true;
            flag_reload_TIM4 = true;
            selected_item++;
          }
          break;

        case (K_4_Pin):
          if (longpress)
          {

          }
          else
          {
            calibration_correction = LIM_CALIBRATION_RANGEWIDTH / 2;
            flag_reload_TIM3 = true;
            flag_reload_TIM4 = true;
            selected_item--;
          }
          break;

        case (EN1_button_Pin):
          if (longpress)
          {

          }
          else
          {

          }
          break;

        case (EN2_button_Pin):
          if (longpress)
          {

          }
          else
          {

          }
          break;

      }
      break;
    //*********************************************************************************************
  }

  switch (key_number)
  {
    case (EXT_SENSOR_Pin):

      break;
    case (IRON_SENSOR_Pin):
      if (IRON.sensor)
        flag_IRON_WAKEUP = true;
      break;
  }

}
//*****************************************************************************

//*****************************************************************************
void encoder_uploader (TIM_HandleTypeDef *htim, bool *flag, uint16_t *set, uint16_t min, uint16_t max, uint8_t step)
{
  if (*set < min)
    *set = min;
  if (*set > max)
    *set = max;
  if (*flag)
  {
    htim->Instance->ARR = (max - min) / step * 4;
    htim->Instance->CNT = (*set - min) / step * 4;
    *flag = false;
  }
  *set = (__HAL_TIM_GET_COUNTER(htim) * step / 4) + min;
}
//*****************************************************************************

//*****************************************************************************
float get_K (uint16_t K)
{
  float f_K = (float)(K - 100)/500 + 1;
  return f_K;
}

//*****************************************************************************
uint32_t approximateT (uint16_t ADC_4, uint16_t ADC_3, uint16_t ADC_2, uint16_t T_3, uint16_t T_2)
{
  return (ADC_4 - ADC_3)*(T_3 - T_2)/(ADC_3 - ADC_2) + T_3;
}
//*****************************************************************************

//*****************************************************************************
float get_NTC_T (uint16_t table, float K, uint16_t adc)
{
  float temperature = 0;
  switch (table)
  {
    case (1):
      temperature = round (ADC_converter (K * adc, MF52AT_10K_B3950, TABLE_SIZE(MF52AT_10K_B3950)));
      break;
    case (2):
      temperature = round (ADC_converter (K * adc, COMMON_NTC_10K_B3435, TABLE_SIZE(COMMON_NTC_10K_B3435)));
      break;
    case (3):
      temperature = round (ADC_converter (K * adc, COMMON_NTC_10K_B3950, TABLE_SIZE(COMMON_NTC_10K_B3950)));
      break;
    case (4):
      temperature = round (ADC_converter (K * adc, COMMON_NTC_10K_B3988, TABLE_SIZE(COMMON_NTC_10K_B3988)));
      break;
  }

  return temperature;
}
//*****************************************************************************


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq ();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
