/***********************************************************************************************************

  Author:     Am0k
  Github:     https://github.com/Am0k-GIT

***********************************************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

#include "ring_buffer.h"

bool ascending(uint16_t a, uint16_t b)
{
    return a > b;
}

bool ascendingABS(int16_t a, int16_t b)
{
    return abs(a) > abs(b);
}

bool descending(uint16_t a, uint16_t b)
{
    return a < b;
}

bool descendingABS(int16_t a, int16_t b)
{
    return abs(a) < abs(b);
}

void sortArray(int16_t* array, uint8_t size, bool (*comparisonFcn) (int16_t, int16_t))
{
    for (uint8_t startIndex = 0; startIndex < size; ++startIndex)
    {
        uint8_t bestIndex = startIndex;
        for (uint8_t currentIndex = startIndex + 1; currentIndex < size; ++currentIndex)
        {
            if (comparisonFcn(array[bestIndex], array[currentIndex]))
                bestIndex = currentIndex;
        }
        int16_t buf = array[startIndex];
        array[startIndex] = array[bestIndex];
        array[bestIndex] = buf;
    }
}

void RING_BUF_Init(RING_buffer_t* Handle, uint8_t size)
{
    Handle->_m_size = size;
    Handle->_ADC_RW_array = malloc(size * sizeof(uint16_t));
    memset(Handle->_ADC_RW_array, 0, size * sizeof(uint16_t));
    Handle->_m_index = 0;
    Handle->_m_full = false;
}

void RING_BUF_Push(RING_buffer_t* Handle, uint16_t value)
{
    Handle->_ADC_RW_array[Handle->_m_index] = value;
    if (Handle->_m_index < Handle->_m_size - 1)
    {
        Handle->_m_index++;
    }
    else
    {
        Handle->_m_index = 0;
        Handle->_m_full = true;
    }
}

int16_t RING_BUF_getFiltered(RING_buffer_t* Handle, uint8_t percent)
{
    uint16_t average = 0;
    uint8_t size = 0;
    if (Handle->_m_full)
        size = Handle->_m_size;
    else
        size = Handle->_m_index;

    for (uint8_t currentIndex = 0; currentIndex < size; currentIndex++)
    {
        average += Handle->_ADC_RW_array[currentIndex];
    }
    average = round(average / size);

    int16_t* errors_array = malloc((size) * sizeof(int16_t));
    memset(errors_array, 0, (size) * sizeof(int16_t));

    for (uint8_t currentIndex = 0; currentIndex < size; currentIndex++)
    {
        errors_array[currentIndex] = average - Handle->_ADC_RW_array[currentIndex];
    }

    sortArray(errors_array, size, ascendingABS);

    uint8_t new_size = round((float)percent / 100 * (size));
    if (new_size < 1)
        new_size = 1;

    int16_t errors_sum = 0;
    for (uint8_t currentIndex = 0; currentIndex < new_size; currentIndex++)
    {
        errors_sum += errors_array[currentIndex];
    }

    free(errors_array);

    return round(average - errors_sum / new_size);
}

#ifdef __cplusplus
}
#endif
