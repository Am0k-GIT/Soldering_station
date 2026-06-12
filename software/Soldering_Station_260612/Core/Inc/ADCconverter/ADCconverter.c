/***********************************************************************************************************

  Author:     Am0k
  Github:     https://github.com/Am0k-GIT

***********************************************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

#include "ADCconverter.h"

float ADC_converter(raw_value_t current_raw_value, calibration_entry_t table[], uint8_t size)
{
    bool ascending_sort = (table[0].raw_value < table[size - 1].raw_value);    // определяем направление сортировки
    // проверка границ с ранним выходом
    if (ascending_sort)
    {
        if (current_raw_value <= table[0].raw_value)
            return table[0].proc_value;
        if (current_raw_value >= table[size - 1].raw_value)
            return table[size - 1].proc_value;
    }
    else
    {
        if (current_raw_value >= table[0].raw_value)
            return table[0].proc_value;
        if (current_raw_value <= table[size - 1].raw_value)
            return table[size - 1].proc_value;
    }

    uint8_t left = 0;
    uint8_t right = size - 1;
    while (right - left > 1)
    {
        uint8_t mid = (left + right) >> 1;
        if ((current_raw_value < table[mid].raw_value) == ascending_sort)
        {
            right = mid;
        }
        else
        {
            left = mid;
        }
    }
    raw_value_t vl = table[left].raw_value;
    raw_value_t vr = table[right].raw_value;
    proc_value_t pl = table[left].proc_value;
    proc_value_t pr = table[right].proc_value;

    float delta_raw = vr - vl;
    float offset = current_raw_value - vl;

    return pl + (pr - pl) * offset / delta_raw;
}

float GetVoltageRVD(uint16_t ADC_CURRENT, uint16_t ADC_FULL, float Vref, uint32_t R_UP, uint32_t R_DOWN)
{
    return (ADC_CURRENT * Vref * (R_UP + R_DOWN)) / (ADC_FULL * R_DOWN);
}


#ifdef __cplusplus
}
#endif
