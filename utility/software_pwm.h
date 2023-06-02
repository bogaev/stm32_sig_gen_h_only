#ifndef _SOFTWARE_PWM_H_
#define _SOFTWARE_PWM_H_

#include "utility\timer_facility.hpp"
#include "utility\callback.hpp"

#include <memory>

namespace util {
  
struct FreqInitSettings {
  float freq;
  float duty_cycle;
};

struct PeriodsInitSettings {
  uint16_t high_period;
  uint16_t low_period;
};

class SoftwarePWM {
public:
  using ITimer = tim::TimerInterface;
  using OS_Timer = tim::SW_FreeRTOS_Timer;
  using HW_Timer = tim::HW_STM32_BaseTimer;
  enum enPeriod { LOW = 0, HIGH };
  enum TimType { OS, HW };
  
  SoftwarePWM(TimType type, FreqInitSettings settings);
  SoftwarePWM(TimType type, PeriodsInitSettings settings);
  void Start();
  void Stop();
  void SetFrequency(uint16_t freq);
  void SetPeriods(uint16_t high_period, uint16_t low_period);
  template<typename T>
  void SetPwmPeriodChangeHandler(T* owner, void (T::*func_ptr)(enPeriod));
 
private:
  void InitTimer(TimType type);
  void ExpireTimerHandler();
  
  FreqInitSettings settings_;
  std::unique_ptr<_util::GenericCallback<enPeriod>> period_change_callback_;
  std::unique_ptr<ITimer> timer_;
  TimerPeriod high_period_{0_ms};
  TimerPeriod low_period_{0_ms};
  enPeriod current_period_ = HIGH;
};

template<typename T>
void SoftwarePWM::SetPwmPeriodChangeHandler(T* owner, void (T::*func_ptr)(enPeriod)) {
  period_change_callback_ = std::make_unique<_util::Callback<T, enPeriod>>(owner, func_ptr);
}

} // namespace _util

#endif // _SOFTWARE_PWM_H_