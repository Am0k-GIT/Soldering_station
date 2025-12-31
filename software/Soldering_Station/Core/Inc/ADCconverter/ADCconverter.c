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
    uint8_t left_index = 0;                                                    // самый левый индекс КТ
    uint8_t right_index = size - 1;                                            // правый индекс КТ
    bool ascending_sort = false;                                               // метод сортировки
    if (table[left_index].raw_value < table[right_index].raw_value)            // если таблица идет по возрастанию
        ascending_sort = true;                                                 // используем сортировку по возрастанию
        // получаем сырые показания
    if (ascending_sort)                                                        // если сортировка по возрастанию
    {
        if (current_raw_value <= table[left_index].raw_value)                  // проверяем левый предел
            return table[left_index].proc_value;                               // подгоняем значение под крайнее левое
        if (current_raw_value >= table[right_index].raw_value)                 // проверяем правый предел
            return table[right_index].proc_value;                              // подгоняем значение под крайнее правое
    }
    else                                                                       // если сортировка по убыванию
    {
        if (current_raw_value >= table[left_index].raw_value)                  // проверяем левый предел
            return table[left_index].proc_value;                               // подгоняем значение под крайнее левое
        if (current_raw_value <= table[right_index].raw_value)                 // проверяем правый предел
            return table[right_index].proc_value;                              // подгоняем значение под крайнее правое
    }
    while ((right_index - left_index) > 1)                                     // пока не получим минимальный интервал в таблице
    {
        uint8_t mid_index = (left_index + right_index) >> 1;                   // находим середину
        raw_value_t mid = table[mid_index].raw_value;                          // читаем среднее сырое значение из таблицы

        if (((current_raw_value > mid) && (!ascending_sort)) || ((current_raw_value < mid) && (ascending_sort)))
        // если показания АЦП больше среднего значения из таблицы
        {
            right_index = mid_index;                                           // смещаем правый индекс
        }
        else                                                                   // иначе
        {
            left_index = mid_index;                                            // смещаем левый индекс
        }
    }                                                                          // теперь в минимальное возможном интервале
    raw_value_t vl = table[left_index].raw_value;                              // читаем из таблицы левое сырое значение
    proc_value_t vl_proc = table[left_index].proc_value;                       // и соответствующее конвертированное значение
    if (((current_raw_value >= vl) && (!ascending_sort)) || ((current_raw_value <= vl) && (ascending_sort)))
    // если показания АЦП больше или равны левому сырому значению
    {
        return vl_proc;                                                        // возвращаем соответствующее конвертированное значение
    }
    raw_value_t vr = table[right_index].raw_value;                             // читаем из таблицы правое сырое значение
    raw_value_t vd = abs(vl - vr);                                             // подсчитываем разность сырых значений
    float res = table[right_index].proc_value;                                 // в результат записываем правое конвертированное значение
    if (vd)                                                                    // если  разность сырых значений не нулевая
    {
        res -= ((res - vl_proc) * abs(current_raw_value - vr) / vd);
        // проводим линейную интерполяцию для поиска смещения конвертированного значения
    }                                                                          // и вычитаем из результата
    return res;                                                                // возвращаем результат
}

float GetVoltageRVD(uint16_t ADC_CURRENT, uint16_t ADC_FULL, float Vref, uint32_t R_UP, uint32_t R_DOWN)
{
    return (ADC_CURRENT * Vref * (R_UP + R_DOWN)) / (ADC_FULL * R_DOWN);
}


#ifdef __cplusplus
}
#endif
