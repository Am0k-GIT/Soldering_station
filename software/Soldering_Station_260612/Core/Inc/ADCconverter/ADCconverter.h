/***********************************************************************************************************

  Author:     Am0k
  Github:     https://github.com/Am0k-GIT

***********************************************************************************************************/

#ifndef ADC_converter_H
#define ADC_converter_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

typedef uint16_t raw_value_t;                                                  // тип сырых показаний АЦП
typedef int16_t proc_value_t;                                                  // тип конвертированных в градусы Цельсия показаний

typedef struct
{
    raw_value_t raw_value;
    proc_value_t proc_value;
} calibration_entry_t;
// структура соответствующих пар сырых показаний / реальных значений

#define TABLE_SIZE(x) (sizeof(x)/sizeof((x)[0]))

float ADC_converter(raw_value_t current_raw_value, calibration_entry_t table[], uint8_t size);

float GetVoltageRVD(uint16_t ADC_CURRENT, uint16_t ADC_FULL, float Vref, uint32_t R_UP, uint32_t R_DOWN);

#ifdef __cplusplus
}
#endif

#endif

