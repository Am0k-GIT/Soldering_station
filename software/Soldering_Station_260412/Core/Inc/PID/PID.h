/***********************************************************************************************************

  Author:     Am0k
  Github:     https://github.com/Am0k-GIT

***********************************************************************************************************/

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

typedef struct {
    int16_t PID_MIN;                                                           // минимальный выход регулятора
    int16_t PID_MAX;                                                           // максимальный выход регулятора
    float _Imax;                                                               // максимальное значение составляющей I (в долях от 1)
    float _Kp;                                                                 // коэф P
    float _Ki;                                                                 // коэф I
    float _Kd;                                                                 // коэф D
    float _Ki_sync;                                                            // коэф I
    float _Kd_sync;                                                            // коэф D
    uint32_t _timeStep;
    uint32_t _timePrevious;                                                    // предыдщая отметка времени
    int16_t _result;                                                           // выходной сигнал регулятора
    bool _invert;                                                              // инверсия (больше воздействие - меньше установовшееся значение)
    int16_t _setValue;                                                         // установившееся значение, которое нужно получить
    float _errorPrevious;                                                      // предыдущая ошибка
    float _resIntegral_buffer;                                                 // буфер для накопления интегральной ошибки
    int16_t _resProporcional;                                                  // P составляющая выходного сигнала
    int16_t _resIntegral;                                                      // I составляющая выходного сигнала
    int16_t _resDifferential;                                                  // D составляющая выходного сигнала
} PID_t;

void PID_Init (PID_t *Handle, bool invert);
void PID_Set_K (PID_t *Handle, float Kp, float Ki, float Kd, float Imax);
void PID_SetLimits (PID_t *Handle, int16_t min, int16_t max);
void PID_SetSync (PID_t *Handle, int32_t timeStep);
void PID_SetValue (PID_t *Handle, int16_t value);
void PID_Restart (PID_t *Handle);
int16_t PID_Get_P(PID_t *Handle);
int16_t PID_Get_I(PID_t *Handle);
int16_t PID_Get_D(PID_t *Handle);
int16_t PID_GetResultAsync (PID_t *Handle, float value, int32_t time);
int16_t PID_GetResultSync (PID_t *Handle, float value);

#ifdef __cplusplus
}
#endif

#endif
