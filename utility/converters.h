#ifndef _VALUE_CONVERTERS_H_
#define _VALUE_CONVERTERS_H_

#include "tim.h"

namespace __utility {

inline uint32_t PercentsToTimerValue(TIM_HandleTypeDef* timer, uint32_t value) {
  return uint32_t(timer->Init.Period * float(value) / 100.);
}

} // namespace __utility

#endif // _VALUE_CONVERTERS_H_