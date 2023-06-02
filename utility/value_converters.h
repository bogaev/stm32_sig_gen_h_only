#ifndef _VALUE_CONVERTERS_H_
#define _VALUE_CONVERTERS_H_

#include "tim.h"
#include "tim.h"

//#include "Common\components.hpp"

#include <array>

namespace _util {

inline uint32_t PercentsToTimerValue(TIM_HandleTypeDef* timer, int8_t value) {
  return uint32_t(timer->Init.Period * float(value) / 100.);
}

template<typename Cont>
std::array<float, ENCODERS_NUM> GetBalanceCoefficients(const Cont& v) {
  std::array<float, ENCODERS_NUM> ret;
  
  for(int i = 0; i < v.size(); ++i)
    ret[i] = float(v[0]) / float(v[i]);
    
  return ret;
}

} // namespace _util

#endif // _VALUE_CONVERTERS_H_