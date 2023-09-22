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
#include "signal_generator\signals.h"
#include "utility\observer.h"
#include "utility\shared_objects.h"
#include "app\common.h"

static uint16_t fmax_dbg = 0;
static uint16_t fmin_dbg = 0xFFFF;

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
  FP_TYPE GetFreq() const;
  FP_TYPE GetPeriod() const;
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

  uint8_t amod_depth_percent_ = 100;
  FP_TYPE mod_sig_value_ = 0.f;
  FP_TYPE mod_sig_freq_ = 0.f;
  uint32_t sample_ = 0; // номер текущего семпла (точки) для которого вычислется значение
};

inline SignalModulator::SignalModulator(uint32_t sample_rate)
  : sample_rate_(sample_rate)
  , carrier_(Signal{sample_rate_}.Create(SIG_GEN_TYPE_NONE))
  , amod_(Signal{sample_rate_}.Create(SIG_GEN_TYPE_NONE))
  , fmod_(Signal{sample_rate_}.Create(SIG_GEN_TYPE_NONE)) {
      SharedObjects::signal_updater.RegisterObserver(this);
}

inline SignalModulator::SignalModulator(uint32_t sample_rate,
                                 enSignalTypes carrier,
                                 enSignalTypes amod,
                                 enSignalTypes fmod)
  : sample_rate_(sample_rate)
  , carrier_(Signal{sample_rate_}.Create(carrier))
  , amod_(Signal{sample_rate_}.Create(amod))
  , fmod_(Signal{sample_rate_}.Create(fmod)) {
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
    GenerateCarrier();
  } else {
    mod_sig_value_ = 0.0f; // если не задано никакого сигнала - выводится 0
  }
  ++sample_;
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
inline SignalModulator& SignalModulator::AddAmpMod() {
  mod_sig_value_ *= (amod_->GetValue(sample_) + 1.0) * 0.5
            * ((FP_TYPE)amod_depth_percent_ / 100.0f)
                + (1.0f - (FP_TYPE)amod_depth_percent_ / 100.0f);
  return *this;
}

/**
  * @brief  Вычисляет значение частотно-модулированного сигнала
  */
inline SignalModulator& SignalModulator::GenerateFreqMod() {
  auto [val, freq] = carrier_->FreqMod(sample_, *fmod_);
  mod_sig_value_ = val;
  mod_sig_freq_ = freq;
  return *this;
}

/**
  * @brief  Возвращает значение амплитуды несущего сигнала
  * @retval Значение амплитуды несущего сигнала
  */
inline FP_TYPE SignalModulator::GetAmp() const {
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
inline FP_TYPE SignalModulator::GetFreq() const {
  if (carrier_ && amod_ && fmod_) { // если все сигналы != nullptr
    return mod_sig_freq_;
  } else if (carrier_ && amod_) { // если сигналы carrier_ и amod_ != nullptr
    return carrier_->GetFreq();
  } else if (carrier_ && fmod_) { // если сигналы carrier_ и fmod_ != nullptr
    return mod_sig_freq_;
  } else if (carrier_) { // если задан только несущий сигнал
    return carrier_->GetFreq();
  }
  return 1.0f;
}

inline FP_TYPE SignalModulator::GetPeriod() const {
  return 1.0f / GetFreq();
}

inline enSignalTypes SignalModulator::GetCarrierType() const {
  if (carrier_) {
    return carrier_->GetSignalType();
  }
  return SIG_GEN_TYPE_NONE;
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
  } else if (signal == SIG_GEN_AMP_MOD) {
    if (param == SIG_GEN_PARAM_AMP_DEPTH) {
      amod_depth_percent_ = (uint8_t) value;
      return;
    }
    if (param == SIG_GEN_PARAM_SIGNAL_TYPE) {
      amod_ = Signal(sample_rate_).Create(static_cast<enSignalTypes>(value));
      return;
    }
    if (param == SIG_GEN_PARAM_FREQ) {
      amod_->SetParam(param, value / 2);
      return;
    }
    amod_->SetParam(param, value);
  } else if (signal == SIG_GEN_FREQ_MOD) {
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

inline void SignalModulator::Reset() {
  sample_ = 0;
  mod_sig_value_ = 0.0f;
}

#endif // #ifndef _SIGNAL_MODULATOR_H_
