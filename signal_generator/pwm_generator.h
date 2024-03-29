/**
  ******************************************************************************
  * @file    pwm_generator.h
  * @author  bogaev.s@gmail.com
  * @brief   Файл определяет классы для вычисления ШИМ на основе
  *          модулированного сигнала
  ******************************************************************************
  */

#ifndef _PWM_GENERATOR_H_
#define _PWM_GENERATOR_H_

#include <cmath>
#include <memory>
#include <queue>

#include "main.h"

#include "signal_generator\signal_modulator.h"

namespace pwm_gen {

//const SIG_GEN_RangeCoeff corr_amp[] = {
//  { .from = 0, .to = 3, .coeff = 100.0 }
//};
//
//const SIG_GEN_RangeCoeff corr_freq[] = {
//  { .from = 0, .to = 100, .coeff = 97 },
//  { .from = 100, .to = 150, .coeff = 100 },
//  { .from = 150, .to = 200, .coeff = 103 },
//  { .from = 200, .to = 250, .coeff = 108 },
//  { .from = 250, .to = 300, .coeff = 110 },
//  { .from = 300, .to = 400, .coeff = 113 },
//  { .from = 400, .to = 450, .coeff = 118 },
//  { .from = 450, .to = 600, .coeff = 125 },
//  { .from = 600, .to = 800, .coeff = 140 },
//  { .from = 800, .to = 900, .coeff = 165 },
//  { .from = 900, .to = CARRIER_FREQ_MAX_HZ, .coeff = 185 }
//};
//
//SIG_GEN_CoeffsInitStruct corr_coeff = {
//  corr_amp,
//  1,
//  corr_freq,
//  11
//};

class SignalStabilizer final {
public:
  struct Settings {
    const SIG_GEN_CoeffsInitStruct* coeffs = nullptr;
  };

  SignalStabilizer(const SignalStabilizer::Settings& settings)
    : coeffs_(settings.coeffs),
      max_amp_(coeffs_->amp_array[coeffs_->freq_array_size - 1].to),
      max_freq_(coeffs_->freq_array[coeffs_->freq_array_size - 1].to)
  {}

  FP_TYPE GetAmpCoeff(FP_TYPE current_amp) const {
    return GetCoeff(current_amp, coeffs_->amp_array, coeffs_->amp_array_size);
  }

  FP_TYPE GetFreqCoeff(FP_TYPE current_freq) const {
    return GetCoeff(current_freq, coeffs_->freq_array, coeffs_->freq_array_size);
  }

 private:
  FP_TYPE GetCoeff(FP_TYPE value, const SIG_GEN_RangeCoeff* array, uint32_t size) const {
    for (uint32_t i = 0; i < size; ++i) {
      if (value >= array[i].from && value < array[i].to) {
        return array[i].coeff / 100.0f;
      }
      if (i == size-1 && value >= array[i].from && value <= array[i].to) {
        return array[i].coeff / 100.0f;
      }
    }
    return 1.0f;
  }

  const SIG_GEN_CoeffsInitStruct* coeffs_;
  const FP_TYPE max_amp_;
  const FP_TYPE max_freq_;
};

/**
  * @brief  Вычисляет ШИМ на основе модулированного сигнала
  */
class PwmGenerator final {
 public:
  struct Settings {
    FP_TYPE min_percent;
    FP_TYPE max_percent;
    FP_TYPE timer_period;
  };

  PwmGenerator(SignalModulator sig_mod,
               const PwmGenerator::Settings& duty_cycle_settings,
               const SignalStabilizer::Settings& stabilizer_settings,
               uint8_t dead_time_th_percent);

  void SetSignal(uint8_t signal, uint8_t param, FP_TYPE value);
  uint32_t GetValue();
  bool IsNegHalfwave() const;

 private:
  struct tdSignalParamsBuffer {
    uint8_t signal = 0;
    uint8_t param = 0;
    FP_TYPE value = 0.0f;
  };

  void CommitChanges(SignalModulator& sig_gen) {
    auto p = buffer_params_.front();
    sig_gen.SetSignal(p.signal, p.param, p.value);
    buffer_params_.pop();
  }

  void BufferParams(uint8_t signal, uint8_t param, FP_TYPE value) {
    buffer_params_.push({ signal, param, value });
  }

  FP_TYPE GetMinDutyCycle() const;
  FP_TYPE GetMaxDutyCycle() const;
  FP_TYPE GetRangeDutyCycle() const;
  void CheckDeadTimeBasedOnPeriod();
  void CheckDeadTimeBasedOnAmpl(FP_TYPE value);
  void Reset();

  SignalModulator sig_mod_; /// результирующий модулированный сигнал для вычисления ШИМ
  const FP_TYPE min_duty_cycle_;
  const FP_TYPE max_duty_cycle_;
  const std::shared_ptr<SignalStabilizer> stabilizer_;
  const FP_TYPE dead_time_th_ = 0.0f;
  const uint32_t sample_rate_ = SAMPLE_RATE;
  bool is_neg_halfwave_ = false;
  FP_TYPE pwm_value_ = 0.0f;
  std::queue<tdSignalParamsBuffer> buffer_params_;
};

inline PwmGenerator::PwmGenerator(SignalModulator sig_mod,
                       const PwmGenerator::Settings& duty_cycle_settings,
                       const SignalStabilizer::Settings& stabilizer_settings,
                       uint8_t dead_time_th_percent)
  : sig_mod_(std::move(sig_mod))
  , min_duty_cycle_(duty_cycle_settings.timer_period * duty_cycle_settings.min_percent / 100.0f)
  , max_duty_cycle_(duty_cycle_settings.timer_period * duty_cycle_settings.max_percent / 100.0f)
  , stabilizer_(stabilizer_settings.coeffs ?
                  std::make_unique<SignalStabilizer>(stabilizer_settings) : nullptr)
  , dead_time_th_((FP_TYPE)dead_time_th_percent / 100.0f)
{}

/**
  * @brief  Изменение значения параметра сигнала
  */
inline void PwmGenerator::SetSignal(uint8_t signal, uint8_t param, FP_TYPE value) {
  // Reset();
  sig_mod_.SetSignal(signal, param, value);
}

/**
  * @brief  Вычисляет значение ШИМ в следующей точке
  * @retval Следующее значение ШИМ-сигнала
  */
inline uint32_t PwmGenerator::GetValue() {
  pwm_value_ = sig_mod_.GetValue();

  if (     (pwm_value_ >= 0.0f && is_neg_halfwave_)
        || (pwm_value_ <= 0.0f && !is_neg_halfwave_)) {
    is_neg_halfwave_ = !is_neg_halfwave_;
  }

  if (sig_mod_.GetCarrierType() == SIG_GEN_TYPE_SINUS) {
    CheckDeadTimeBasedOnAmpl(pwm_value_);
  } else {
    CheckDeadTimeBasedOnPeriod();
  }

  return (uint32_t)(GetMinDutyCycle() + std::abs(GetRangeDutyCycle() * pwm_value_));
}

/**
  * @brief  Возвращает текущую полярность ШИМ-сигнала
  * @retval true - для отрицательной полуволны, false - для положительной
  */
inline bool PwmGenerator::IsNegHalfwave() const {
  return is_neg_halfwave_;
}

inline FP_TYPE PwmGenerator::GetMinDutyCycle() const {
  return min_duty_cycle_;
//    * (stabilizer_ ? stabilizer_->GetFreqCoeff(sig_mod_.GetFreq()) : 1.0f);
}

inline FP_TYPE PwmGenerator::GetMaxDutyCycle() const {
  return max_duty_cycle_
    * (stabilizer_ ? stabilizer_->GetFreqCoeff(sig_mod_.GetFreq()) : 1.0f);
}

inline FP_TYPE PwmGenerator::GetRangeDutyCycle() const {
  return (GetMaxDutyCycle() - GetMinDutyCycle())
    * (stabilizer_ ? stabilizer_->GetAmpCoeff(sig_mod_.GetAmp()) : 1.0f);
}

inline void PwmGenerator::CheckDeadTimeBasedOnPeriod() {
  uint32_t period = (uint32_t)(1.0f / sig_mod_.GetFreq() * (FP_TYPE)sample_rate_);
  uint32_t sample = sig_mod_.GetSampleNum();
  uint32_t t = sample % period;
  const FP_TYPE dead_time = dead_time_th_ / 4.0f;

  if (	 (FP_TYPE)t / (FP_TYPE)period < dead_time
      || (FP_TYPE)t / (FP_TYPE)period > (1.0f - dead_time)) {
    pwm_value_ = 0.0f;
  }

  if (	 (FP_TYPE)t / (FP_TYPE)period > 0.5f - dead_time
      && (FP_TYPE)t / (FP_TYPE)period < 0.5f + dead_time) {
    pwm_value_ = 0.0f;
  }
}

inline void PwmGenerator::CheckDeadTimeBasedOnAmpl(FP_TYPE value) {
  if (std::abs(value / sig_mod_.GetAmp()) < dead_time_th_) {
    pwm_value_ = 0.0f;
  }
}

/**
  * @brief  Сбрасывает значение текущего сигнала перед сменой параметров
  */
inline void PwmGenerator::Reset() {
  sig_mod_.Reset();
  is_neg_halfwave_ = false;
}

}

#endif // #ifndef _PWM_GENERATOR_H_
