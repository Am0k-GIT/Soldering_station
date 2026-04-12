/***********************************************************************************************************

  Author:     Am0k
  Github:     https://github.com/Am0k-GIT

***********************************************************************************************************/

#ifndef INC_RING_BUFFER_H_
#define INC_RING_BUFFER_H_

#define RING_BUF_MAX_SIZE 64

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
	int16_t* _RW_array;
	int16_t* _ERR_array;
	uint8_t _m_index;
	uint8_t _m_size;
	bool _m_full;
} RING_buffer_t;

void RING_BUF_Init(RING_buffer_t* Handle, uint8_t size);
void RING_BUF_Push(RING_buffer_t* Handle, int16_t value);
int16_t RING_BUF_getFiltered(RING_buffer_t* Handle, uint8_t maxDiffPercent);

#ifdef __cplusplus
}
#endif

#endif /* INC_ADCCONVERTER_RING_BUFFER_RING_BUFFER_H_ */
