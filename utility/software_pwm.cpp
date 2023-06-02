#include "software_pwm.hpp"

namespace util {
  
SoftwarePWM::SoftwarePWM(TimType type, FreqInitSettings settings)
  : settings_(settings)
{
  assert_param(settings_.freq > 0.);
  assert_param(settings_.duty_cycle > 0. && settings_.duty_cycle < 1.);
  SetFrequency((int)settings_.freq);
  InitTimer(type);
}

SoftwarePWM::SoftwarePWM(TimType type, PeriodsInitSettings settings) {
  assert_param(settings.high_period > 0);
  assert_param(settings.low_period > 0);
  SetPeriods(settings.high_period, settings.low_period);
  InitTimer(type);
}

void SoftwarePWM::Start() {
  timer_->Start();
}

void SoftwarePWM::Stop() {
  timer_->Stop();
}

void SoftwarePWM::SetFrequency(uint16_t freq) {
  settings_.freq = (float)freq;
  high_period_ = (uint16_t) (1. / settings_.freq * settings_.duty_cycle * 1000.);
  low_period_ = (uint16_t) (1. / settings_.freq * (1.-settings_.duty_cycle) * 1000.);
}

void SoftwarePWM::SetPeriods(uint16_t high_period, uint16_t low_period) {
  high_period_ = high_period;
  low_period_ = low_period;
}

// private section
void SoftwarePWM::InitTimer(TimType type) {
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

void SoftwarePWM::ExpireTimerHandler() {
  current_period_ = (current_period_ == HIGH ? LOW : HIGH);
  TimerPeriod period_time = (current_period_ == HIGH ? high_period_ : low_period_);
  timer_->ChangePeriod(period_time);
  timer_->Reset();
  if (period_change_callback_->isValid())
    period_change_callback_->execute(current_period_);
}
  
} // namespace _util
