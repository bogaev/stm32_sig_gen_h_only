#ifndef _SOFTWARE_PWM_H_
#define _SOFTWARE_PWM_H_

#include "utility/timer_facility.h"
#include "utility/callback.h"

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

inline SoftwarePWM::SoftwarePWM(TimType type, FreqInitSettings settings)
    : settings_(settings) {
  assert_param(settings_.freq > 0.);
  assert_param(settings_.duty_cycle > 0. && settings_.duty_cycle < 1.);
  SetFrequency((int)settings_.freq);
  InitTimer(type);
}

inline SoftwarePWM::SoftwarePWM(TimType type, PeriodsInitSettings settings) {
  assert_param(settings.high_period > 0);
  assert_param(settings.low_period > 0);
  SetPeriods(settings.high_period, settings.low_period);
  InitTimer(type);
}

inline void SoftwarePWM::Start() {
  timer_->Start();
}

inline void SoftwarePWM::Stop() {
  timer_->Stop();
}

inline void SoftwarePWM::SetFrequency(uint16_t freq) {
  settings_.freq = (float)freq;
  high_period_ = (uint16_t) (1. / settings_.freq * settings_.duty_cycle * 1000.);
  low_period_ = (uint16_t) (1. / settings_.freq * (1.-settings_.duty_cycle) * 1000.);
}

inline void SoftwarePWM::SetPeriods(uint16_t high_period, uint16_t low_period) {
  high_period_ = high_period;
  low_period_ = low_period;
}

// private section
inline void SoftwarePWM::InitTimer(TimType type) {
  if(type == OS) {
    timer_ = std::unique_ptr<ITimer>(new OS_Timer(this, 1_ms, AUTORELOAD_ON));
  } else {
    timer_ = std::unique_ptr<ITimer>(new HW_Timer(USER_HTIM, 1_ms));
  }
  timer_->SetExpireTimerHandler(this, &SoftwarePWM::ExpireTimerHandler);
  TimerPeriod period_time = (current_period_ == HIGH ? high_period_ : low_period_);
  timer_->ChangePeriod(period_time);
  timer_->Reset();
}

inline void SoftwarePWM::ExpireTimerHandler() {
  current_period_ = (current_period_ == HIGH ? LOW : HIGH);
  TimerPeriod period_time = (current_period_ == HIGH ? high_period_ : low_period_);
  timer_->ChangePeriod(period_time);
  timer_->Reset();
  if (period_change_callback_->isValid())
    period_change_callback_->execute(current_period_);
}

} // namespace _util

#endif // _SOFTWARE_PWM_H_