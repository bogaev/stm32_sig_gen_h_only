/**
  ******************************************************************************
  * @file    signal_modulator.h
  * @author  bogaev.s@gmail.com
  * @brief   Файл определяет класс - композицию из 3 сигналов
  *          для вычисления результирующего модулированного сигнала
  ******************************************************************************
  */ 

#ifndef _SIGNAL_MODULATOR_H_
#define _SIGNAL_MODULATOR_H_

#include <cmath>
#include <utility>
#include <memory>

#include "main.h"
#include "signal_generator\sig_gen_include.h"
#include "signal_generator\signals.hpp"
#include "utility\observer.hpp"
#include "utility\shared_objects.hpp"
#include "common_\common.h"

namespace pwm_gen {
  class PwmGenerator;
}

/**
  * @brief  Вычисление значения модулированного сигнала
  */
class SignalModulator : public _pattern::Observer<tdSignalParams> {  
 public:
  friend class pwm_gen::PwmGenerator;
  
  SignalModulator(uint32_t sample_rate);
  
  explicit SignalModulator(uint32_t sample_rate,
                           enSignalTypes carrier,
                           enSignalTypes amod = SIG_GEN_TYPE_NONE,
                           enSignalTypes fmod = SIG_GEN_TYPE_NONE);
  SignalModulator(SignalModulator&& other) = default;
  virtual ~SignalModulator();
  
  FP_TYPE GetValue();
  FP_TYPE GetAmp() const;
  FP_TYPE GetFreq(enSignals signal = SIG_GEN_CARRIER) const;
  FP_TYPE GetPeriod(enSignals signal = SIG_GEN_CARRIER) const;
  enSignalTypes GetCarrierType() const;
  void SetSignal(uint8_t signal, uint8_t param, FP_TYPE value);
  void Update(tdSignalParams msg) override;
  
 private:
  SignalModulator& GenerateCarrier();
  SignalModulator& AddAmpMod();
  SignalModulator& GenerateFreqMod();
  void Reset();
  uint32_t GetSampleNum() const;
  
  const uint32_t sample_rate_; /// частота семплирования сигнала
  std::unique_ptr<Signal> carrier_; /// сигнал с параметрами несущего сигнала
  std::unique_ptr<Signal> amod_; /// сигнал с параметрами частотной модуляции
  std::unique_ptr<Signal> fmod_; /// сигнал с параметрами частотной модуляции
//  uint32_t amp_mod_period_;
  uint8_t amod_depth_percent_ = 100;
  FP_TYPE mod_sig_value_ = 0.0;
  uint32_t sample_ = 0; // номер текущего семпла (точки) для которого вычислется значение
};

inline SignalModulator::SignalModulator(uint32_t sample_rate) 
  : sample_rate_(sample_rate)
//  , carrier_(Signal{sample_rate_}.Create(SIG_GEN_TYPE_SINUS)) //!!
  , carrier_(Signal{sample_rate_}.Create(SIG_GEN_TYPE_NONE))
  , amod_(Signal{sample_rate_}.Create(SIG_GEN_TYPE_NONE))
  , fmod_(Signal{sample_rate_}.Create(SIG_GEN_TYPE_NONE))
//  , amp_mod_period_(1)
{
  SharedObjects::signal_updater.RegisterObserver(this);
}

inline SignalModulator::SignalModulator(uint32_t sample_rate,
                                 enSignalTypes carrier,
                                 enSignalTypes amod,
                                 enSignalTypes fmod)
  : sample_rate_(sample_rate)
  , carrier_(Signal{sample_rate_}.Create(carrier))
  , amod_(Signal{sample_rate_}.Create(amod))
  , fmod_(Signal{sample_rate_}.Create(fmod))
//  , amp_mod_period_(amod_ ? uint32_t((1.f / amod_->GetFreq()) * sample_rate_) : 1)
{
  SharedObjects::signal_updater.RegisterObserver(this);
}

inline SignalModulator::~SignalModulator() {
  SharedObjects::signal_updater.RemoveObserver(this);
};

inline void SignalModulator::Update(tdSignalParams msg) {
  SetSignal(msg.signal, msg.param, msg.value);
}

/**
  * @brief  Вычисляет отдельное значение результирующего 
  *         модулированного сигнала
  * @retval Значение сигнала в точке sample_
  */
//uint32_t cycles_num = 0;
inline FP_TYPE SignalModulator::GetValue() {
  if (carrier_ && amod_ && fmod_) { // если все сигналы != nullptr
    GenerateFreqMod().AddAmpMod();
  } else if (carrier_ && amod_) { // если сигналы carrier_ и amod_ != nullptr
    GenerateCarrier().AddAmpMod();
  } else if (carrier_ && fmod_) { // если сигналы carrier_ и fmod_ != nullptr
    GenerateFreqMod();
  } else if (carrier_) { // если задан только несущий сигнал
//    StartCyclesCounter();
    GenerateCarrier();
//    cycles_num = GetCyclesCounter();
//    StopCyclesCounter();
  } else {
    mod_sig_value_ = 0.0; // если не задано никакого сигнала - выводится 0
  }
  ++sample_;
//  if (sample_ > 0 && (sample_ % amp_mod_period_) < uint32_t(amp_mod_period_ * 0.01)) {
//    SharedObjects::mod_period_end.NotifyObservers();
//  }
  return mod_sig_value_;
}

/**
  * @brief  Вычисляет значение несущего сигнала
  */
inline SignalModulator& SignalModulator::GenerateCarrier() {
  mod_sig_value_ = carrier_->GetValue(sample_);
  return *this;
}

/**
  * @brief  Добавляет амплитудную модуляцию к несущему сигналу
  *         с учетом глубины модуляции (amod_depth_percent_)
  */
inline SignalModulator& SignalModulator::AddAmpMod()
{
  mod_sig_value_ *= (std::abs(amod_->GetValue(sample_))
            * ((FP_TYPE)amod_depth_percent_ / 100.0)
                + (1.0 - (FP_TYPE)amod_depth_percent_ / 100.0));
  return *this;
}

/**
  * @brief  Вычисляет значение частотно-модулированного сигнала
  */
inline SignalModulator& SignalModulator::GenerateFreqMod()
{
  mod_sig_value_ = carrier_->FreqMod(sample_, *fmod_);
  return *this;
}

/**
  * @brief  Возвращает значение амплитуды несущего сигнала
  * @retval Значение амплитуды несущего сигнала
  */
inline FP_TYPE SignalModulator::GetAmp() const
{
  // TODO
  return carrier_->GetAmp();
}

/**
  * @brief  Возвращает значение частоты сигнала
  *
  *         Возвращает значение частоты одного из сигнала:
  *           несущего, амплитудной модуляции, частотной модуляции
  * @param  signal сигнал, частота которого возвращается
  * @retval Значение частоты выбранного сигнала
  */
inline FP_TYPE SignalModulator::GetFreq(enSignals signal) const
{
  if (signal == SIG_GEN_CARRIER) {
    return carrier_->GetFreq();
  } else if (signal == SIG_GEN_AMP_MOD) {
    return amod_->GetFreq();
  } else if (signal == SIG_GEN_FREQ_MOD) {
    return fmod_->GetFreq();
  }
  return 0.0;
}

inline FP_TYPE SignalModulator::GetPeriod(enSignals signal) const {
  return 1.0 / GetFreq(signal);
}

inline enSignalTypes SignalModulator::GetCarrierType() const {
	return carrier_->GetSignalType();
}

inline uint32_t SignalModulator::GetSampleNum() const {
  return sample_;
}

/**
  * @brief  Устанавливает значение параметра сигнала
  * @param  signal - сигнал, параметр которого изменяется (см. enSignals)
  * @param  param - параметр, который изменяется (см. enSignalParams)
  * @param  value - новое значение параметра сигнала
  */
inline void SignalModulator::SetSignal(uint8_t signal, uint8_t param, FP_TYPE value) {
  Reset();
  if (signal == SIG_GEN_CARRIER) {
    if (param == SIG_GEN_PARAM_SIGNAL_TYPE) {
      carrier_ = Signal(sample_rate_).Create(static_cast<enSignalTypes>(value));
      return;
    }
    carrier_->SetParam(param, value);
  }
	else if (signal == SIG_GEN_AMP_MOD) {
    if (param == SIG_GEN_PARAM_AMP_DEPTH) {
      amod_depth_percent_ = (uint8_t) value;
      return;
    }
    if (param == SIG_GEN_PARAM_SIGNAL_TYPE) {
      amod_ = Signal(sample_rate_).Create(static_cast<enSignalTypes>(value));
//      amp_mod_period_ = uint32_t((1.f / amod_->GetFreq()) * sample_rate_);
      return;
    }
    if (param == SIG_GEN_PARAM_FREQ) {
      amod_->SetParam(param, value/2);
//      amp_mod_period_ = uint32_t((1.f / amod_->GetFreq()) * sample_rate_);
      return;
    }
    amod_->SetParam(param, value);
  }
	else if (signal == SIG_GEN_FREQ_MOD) {
    if (param == SIG_GEN_PARAM_FREQ_DEPTH) {
      carrier_->SetFmodDepth(value);
      return;
    }
    if (param == SIG_GEN_PARAM_SIGNAL_TYPE) {
      fmod_ = Signal(sample_rate_).Create(static_cast<enSignalTypes>(value));
      return;
    }
    if (param == SIG_GEN_PARAM_FREQ) {
      fmod_->SetParam(param, value);
      return;
    }
    fmod_->SetParam(param, value);
  }
}

inline void SignalModulator::Reset()
{
  sample_ = 0;
  mod_sig_value_ = 0.0;
}

#endif // #ifndef _SIGNAL_MODULATOR_H_
