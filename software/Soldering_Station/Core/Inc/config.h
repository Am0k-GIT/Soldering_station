/***********************************************************************************************************

  Author:     Am0k
  Github:     https://github.com/Am0k-GIT

***********************************************************************************************************/

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

//#define DEBUG_UART
#define UART_HEANDLER                            &huart6

#define VERSION                                  251224

#define DEF_IRON_MIN_TEMP                        150
#define DEF_IRON_MAX_TEMP                        450
#define DEF_IRON_STEP_TEMP                       10
#define DEF_IRON_MIN_FLOW                        50
#define DEF_IRON_MAX_FLOW                        100
#define DEF_IRON_STEP_FLOW                       10
#define DEF_IRON_EXT_NTC                         0
#define DEF_IRON_EXT_SENSOR                      0
#define DEF_IRON_SLEEP_TIMER                     120

#define DEF_HOTAIR_MIN_TEMP                      120
#define DEF_HOTAIR_MAX_TEMP                      450
#define DEF_HOTAIR_STEP_TEMP                     10
#define DEF_HOTAIR_MIN_FLOW                      40
#define DEF_HOTAIR_MAX_FLOW                      100
#define DEF_HOTAIR_STEP_FLOW                     10
#define DEF_HOTAIR_EXT_NTC                       0
#define DEF_HOTAIR_EXT_SENSOR                    0
#define DEF_HOTAIR_SLEEP_TIMER                   60

#define DEF_NTC                                  1
#define DEF_ZERO_CURRENT                         2281
#define DEF_K_CURRENT                            100
#define DEF_K_VOLTAGE                            100
#define DEF_K_INT_NTC                            100
#define DEF_K_IRON_NTC                           100
#define DEF_K_HOTAIR_NTC                         100
#define DEF_TIME_OFF                             20*60                         // S
#define DEF_OVERCURRENT_PROTECTION               9                             // A

#define DEF_IRON_TOOL_P                          200
#define DEF_IRON_TOOL_I                          200
#define DEF_IRON_TOOL_D                          200
#define DEF_IRON_TOOL_ADC0                       80                            // T12: 0062 C245: 0080
#define DEF_IRON_TOOL_ADC1                       600                           // T12: 0380 C245: 0600
#define DEF_IRON_TOOL_ADC2                       1330                          // T12: 0850 C245: 1330
#define DEF_IRON_TOOL_ADC3                       2040                          // T12: 1425 C245: 2040
#define DEF_IRON_TOOL_ADC4                       4000                          // T12: 1425 C245: 2720
#define DEF_IRON_TOOL_T0                         0
#define DEF_IRON_TOOL_T1                         120
#define DEF_IRON_TOOL_T2                         240
#define DEF_IRON_TOOL_T3                         360
#define DEF_IRON_TOOL_T4                         (DEF_IRON_TOOL_ADC4 - DEF_IRON_TOOL_ADC3)*(DEF_IRON_TOOL_T3 - DEF_IRON_TOOL_T2)/(DEF_IRON_TOOL_ADC3 - DEF_IRON_TOOL_ADC2) + DEF_IRON_TOOL_T3

#define DEF_HOTAIR_TOOL_P                        200
#define DEF_HOTAIR_TOOL_I                        200
#define DEF_HOTAIR_TOOL_D                        200
#define DEF_HOTAIR_TOOL_ADC0                     80
#define DEF_HOTAIR_TOOL_ADC1                     640
#define DEF_HOTAIR_TOOL_ADC2                     1315
#define DEF_HOTAIR_TOOL_ADC3                     1935
#define DEF_HOTAIR_TOOL_ADC4                     4000
#define DEF_HOTAIR_TOOL_T0                       0
#define DEF_HOTAIR_TOOL_T1                       120
#define DEF_HOTAIR_TOOL_T2                       240
#define DEF_HOTAIR_TOOL_T3                       360
#define DEF_HOTAIR_TOOL_T4                       (DEF_HOTAIR_TOOL_ADC4 - DEF_HOTAIR_TOOL_ADC3)*(DEF_HOTAIR_TOOL_T3 - DEF_HOTAIR_TOOL_T2)/(DEF_HOTAIR_TOOL_ADC3 - DEF_HOTAIR_TOOL_ADC2) + DEF_HOTAIR_TOOL_T3

#define LIM_MIN_TEMP                             100
#define LIM_MAX_TEMP                             500
#define LIM_MIN_STEP_TEMP                        1
#define LIM_MAX_STEP_TEMP                        20
#define LIM_MIN_FAN                              10
#define LIM_MAX_FAN                              100
#define LIM_MIN_STEP_FAN                         1
#define LIM_MAX_STEP_FAN                         25
#define LIM_MIN_SLEEP_TIMER                      0
#define LIM_MAX_SLEEP_TIMER                      600
#define LIM_STEP_SLEEP_TIMER                     10
#define LIM_MIN_PID_K                            0
#define LIM_MAX_PID_K                            1000
#define LIM_STEP_PID_K                           1
#define LIM_MIN_ADC                              0
#define LIM_MAX_ADC                              4095
#define LIM_STEP_ADC                             1
#define LIM_CALIBRATION_RANGEWIDTH               60

#define LOADING_SCREEN_TIME                      3000                          // mS

#define IRON_HOLD_TEMP                           180                           // °C
#define IRON_PREHEAT_POWER                       25                            // %
#define IRON_PREHEAT_P_FACTOR                    0.25
#define IRON_PREHEAT_RANGE                       10                            // °C
#define HOTAIR_HOLD_TEMP                         100                           // °C
#define HOTAIR_SLEEP_FAN                         100                           // %
#define DEEP_SLEEP_TIME                          10*60                         // S
#define OFF_TIME                                 20*60                         // S
#define HOTAIR_TEMP_COOLDOWN                     120                           // °C
#define HOTAIR_BLOW_COOLDOWN                     80                            // %

#define PID_P_IRON_SCALLING                      0.2
#define PID_I_IRON_SCALLING                      0.000005
#define PID_D_IRON_SCALLING                      100

#define PID_P_HOTAIR_SCALLING                    0.1
#define PID_I_HOTAIR_SCALLING                    0.000001
#define PID_D_HOTAIR_SCALLING                    100

#define PID_I_RANGE                              0.25

#define IRON_OVERHEAT_PROTECTION                 50                            // °C
#define HOTAIR_OVERHEAT_PROTECTION               50                            // °C

#define ADC_BUFFER_SIZE                          4
#define ADC_CHANNELS                             8
#define PRECISION                                50                            // %
#define ADC_MAX_VALUE                            4000
#define MEASUREMENT_DELAY                        959                           // uS
#define ADCWORK_DELAY                            41                            // uS
#define PID_IRON_MIN                             (MEASUREMENT_DELAY + ADCWORK_DELAY) * TIM5_COUNT_PER_SEC / 1000000 + 1

#define IRON_PRESETS                             3
#define IRON_TOOLS                               5

#define HOTAIR_PRESETS                           3

#define EEPROM_SIZE                              4096
#define DATA_BLOCK_SIZE                          8
#define EMPTY_BYTE                               0b11111111
#define CTRL_BYTE                                0b01010101

#define EEPROM_IRON_SETTINGS_START               0                             // 18 byte to item
#define EEPROM_IRON_SETTINGS_STOP                255                           // 1 * 256 byte block
#define EEPROM_HOTAIR_SETTINGS_START             256                           // 18 byte to item
#define EEPROM_HOTAIR_SETTINGS_STOP              511                           // 1 * 256 byte block
#define EEPROM_HW_SETTINGS_START                 512                           // 12 byte to item
#define EEPROM_HW_SETTINGS_STOP                  767                           // 1 * 256 byte block
#define EEPROM_IRON_TOOLS_START                  768                           // 26 * TOOLS byte to item
#define EEPROM_IRON_TOOLS_STOP                   3071                          // 9 * 256 byte block
#define EEPROM_HOTAIR_TOOLS_START                3072                          // 26 * TOOLS byte to item
#define EEPROM_HOTAIR_TOOLS_STOP                 3583                          // 2 * 256 byte block
#define EEPROM_IRON_PRESETS_START                3584                          // 4 * PRESETS byte
#define EEPROM_IRON_PRESETS_STOP                 3839                          // 1 * 256 byte block
#define EEPROM_HOTAIR_PRESETS_START              3840                          // 4 * PRESETS byte
#define EEPROM_HOTAIR_PRESETS_STOP               4095                          // 1 * 256 byte block

#define R314                                     10000
#define R315                                     1000

#define ACS712_K                                 0.010959

#define APB1_TIM_FREQ                            84000000                      // TIM2, TIM3, TIM4, TIM5, TIM6, TIM7
#define APB2_TIM_FREQ                            84000000                      // TIM1, TIM8, TIM9, TIM10, TIM11

#define TIM1_FREQ                                1000                          // Hz (20 - 50 000)
#define TIM1_COUNT_PER_SEC                       400000                        // if 1'000'000 one tick = 1uS

#define TIM2_FREQ                                1                             // Hz (1 - 10)
#define TIM2_COUNT_PER_SEC                       1000

#define TIM5_FREQ                                25                            // Hz (20 - 50)
#define TIM5_COUNT_PER_SEC                       50000

#define TIM10_FREQ                               1000                          // dont change !!!
#define TIM10_COUNT_PER_SEC                      1000000                       // if 1'000'000 one tick = 1uS

#define TIM11_FREQ                               10
#define TIM11_COUNT_PER_SEC                      10000

#endif /* INC_CONFIG_H_ */
