/***********************************************************************************************************

  Author:     Am0k
  Github:     https://github.com/Am0k-GIT

***********************************************************************************************************/

#include <PID/PID.h>

int16_t int16_clamp(float d, int16_t min, int16_t max)
{
    if (d < min)
      return min;
    else if (d > max)
      return max;
    else
      return round(d);
}

void PID_Init (PID_t *Handle, bool invert)
{
    Handle->_invert = invert;

    Handle->PID_MIN = 0;
    Handle->PID_MAX = 100;
    Handle->_timePrevious = 0;
    Handle->_result = 0;
    Handle->_setValue = 0;
    Handle->_errorPrevious = 0;
    Handle->_resIntegral = 0;
    Handle->_resDifferential = 0;
}

void PID_Set_K (PID_t *Handle, float Kp, float Ki, float Kd,  float Imax)
{
    Handle->_Kp = Kp;
    Handle->_Ki = Ki;
    Handle->_Kd = Kd;
    Handle->_Imax = Imax;
}

void PID_SetLimits (PID_t *Handle, int16_t min, int16_t max)
{
    Handle->PID_MIN = min;
    Handle->PID_MAX = max;
}

void PID_SetSync (PID_t *Handle, int32_t timeStep)
{
  Handle->_timeStep = timeStep;
}

void PID_SetValue (PID_t *Handle, int16_t value)
{
    Handle->_setValue = value;
}

void PID_Restart (PID_t *Handle)
{
    Handle->_errorPrevious = 0;
    Handle->_resIntegral_buffer = 0;
    Handle->_timePrevious = 0;

    Handle->_Ki_sync = Handle->_Ki * Handle->_timeStep;
    Handle->_Kd_sync = Handle->_Kd / Handle->_timeStep;
}

int16_t PID_Get_P(PID_t *Handle)
{
    return Handle->_resProporcional;
}

int16_t PID_Get_I(PID_t *Handle)
{
    return Handle->_resIntegral;
}

int16_t PID_Get_D(PID_t *Handle)
{
    return Handle->_resDifferential;
}

int16_t PID_GetResultAsync (PID_t *Handle, float value, int32_t time)
{
    float _errorCurrent = 0;
    uint32_t deltaTime = 0;

    if (Handle->_invert)
        _errorCurrent = value - Handle->_setValue;
    else
        _errorCurrent = Handle->_setValue - value;

    Handle->_resProporcional = Handle->_Kp * _errorCurrent;

    Handle->_resProporcional = int16_clamp(Handle->_resProporcional,  0, Handle->PID_MAX);

    if (Handle->_timePrevious)
    {
      deltaTime = (time - Handle->_timePrevious);
      Handle->_timePrevious = time;

      float raw_I_part = Handle->_Ki *_errorCurrent * deltaTime;
      Handle->_resIntegral_buffer += raw_I_part;
      Handle->_resIntegral = int16_clamp(Handle->_resIntegral_buffer, (-1) * (Handle->_Imax) * Handle->PID_MAX, Handle->_Imax * Handle->PID_MAX);

      if ((_errorCurrent > 1) && (Handle->_errorPrevious))
      {
        Handle->_resDifferential = Handle->_Kd * (_errorCurrent - Handle->_errorPrevious) / deltaTime;
        Handle->_resDifferential = int16_clamp(Handle->_resDifferential,  (-1) * (Handle->PID_MAX), Handle->PID_MAX);
      }
      else
      {
        Handle->_resDifferential = 0;
      }
    }

    else
    {
      Handle->_timePrevious = time;

      Handle->_resIntegral = 0;
      Handle->_resDifferential = 0;
    }

    Handle->_result = Handle->_resProporcional + Handle->_resIntegral + Handle->_resDifferential;

    Handle->_result = int16_clamp(Handle->_result,  Handle->PID_MIN, Handle->PID_MAX);

    Handle->_errorPrevious = _errorCurrent;

    return (Handle->_result);
}

int16_t PID_GetResultSync (PID_t *Handle, float value)
{
    float _errorCurrent = 0;

    if (Handle->_invert)
        _errorCurrent = value - Handle->_setValue;
    else
        _errorCurrent = Handle->_setValue - value;

    Handle->_resProporcional = Handle->_Kp * _errorCurrent;

    Handle->_resProporcional = int16_clamp(Handle->_resProporcional,  0, Handle->PID_MAX);


    float raw_I_part = Handle->_Ki_sync *_errorCurrent;
    Handle->_resIntegral_buffer += raw_I_part;
    Handle->_resIntegral = int16_clamp(Handle->_resIntegral_buffer, (-1) * (Handle->_Imax) * Handle->PID_MAX, Handle->_Imax * Handle->PID_MAX);

    if ((_errorCurrent > 1) && (Handle->_errorPrevious))
    {
      Handle->_resDifferential = Handle->_Kd_sync * (_errorCurrent - Handle->_errorPrevious);
      Handle->_resDifferential = int16_clamp(Handle->_resDifferential,  (-1) * (Handle->PID_MAX), Handle->PID_MAX);
    }
    else
    {
      Handle->_resDifferential = 0;
    }

    Handle->_result = Handle->_resProporcional + Handle->_resIntegral + Handle->_resDifferential;

    Handle->_result = int16_clamp(Handle->_result,  Handle->PID_MIN, Handle->PID_MAX);

    Handle->_errorPrevious = _errorCurrent;

    return (Handle->_result);
}
