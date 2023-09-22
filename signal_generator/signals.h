/**
  ******************************************************************************
  * @file    signals.h
  * @author  bogaev.s@gmail.com
  * @brief   Математические функции для вычисления значений сигнала.
  *
  *          Формулы заимствованы отсюда:
  *          https://math.stackexchange.com/questions/178079/
  *          integration-of-sawtooth-square-and-triangle-wave-functions
  ******************************************************************************
  */

#ifndef _SIGNALS_H_
#define _SIGNALS_H_

#include <cmath>
#include <memory>
#include <utility>

/**
  * @brief  Класс абстрактного сигнала
  */
class Signal {
 public:
  Signal(uint32_t sample_timer_period);
  virtual ~Signal() {};
  void SetParam(uint8_t param, FP_TYPE value);

  Signal& SetAmp(FP_TYPE amp);
  Signal& SetFreq(FP_TYPE freq);
  Signal& SetFmodDepth(FP_TYPE mod_depth_percent);

  FP_TYPE GetAmp() const;
  FP_TYPE GetFreq() const;
  FP_TYPE GetModDepth() const;

  std::unique_ptr<Signal> Create(enSignalTypes sig_type);

  /**
    * @brief  Вычисляет отдельное значение сигнала
    * @param  point порядковый номер отсчета сигнала
    * @retval Значение сигнала в точке point
    */
  virtual FP_TYPE GetValue(uint32_t point) const {
    return 0.0f;
  }
  /**
    * @brief  Вычисляет частотную модуляцию сигнала на базе 2 сигналов
    * @param  point порядковый номер отсчета сигнала
    * @param  sig сигнал, которым модулируется частота текущего сигнала
    * @retval Значение сигнала в точке point
    */
  virtual std::pair<FP_TYPE, FP_TYPE> FreqMod(uint32_t point, Signal& sig) const {
    return std::make_pair(0, 0);
  }
  /**
    * @brief  Вычисляет значение интеграла текущего сигнала.
              Используется для вычисления частотной модуляции.
    * @param  point порядковый номер отсчета сигнала
    * @retval Значение интеграла сигнала в точке point
    */
  virtual FP_TYPE GetIntegral(uint32_t point) const {
    return 0.0f;
  }
  /**
    * @brief  Возвращает тип сигнала
    * @retval Тип сигнала
    */
  virtual inline enSignalTypes GetSignalType() const {
    return SIG_GEN_TYPE_NONE;
  }

 protected:
  const FP_TYPE pi = std::acosf(-1.0f);
  const FP_TYPE sample_rate_; /// частота семплирования сигнала в Гц
  uint8_t fmod_depth_percent_ = 100.0f; /// глубина модуляции в процентах
  FP_TYPE freq_ = 1.0f; /// частота сигнала в герцах
  FP_TYPE period_ = 1.0f; /// период сигнала в секундах
  FP_TYPE amp_ = 1.0f; /// амплитуда сигнала (используются значения между 0.0f и 1.0f)
};

/**
  * @brief  Класс синусоидального сигнала
  */
class Sinus : public Signal {
 public:
  Sinus(uint32_t sample_timer_period);
  FP_TYPE GetValue(uint32_t point) const override;
  std::pair<FP_TYPE, FP_TYPE> FreqMod(uint32_t point, Signal& fmod) const override;
  FP_TYPE GetIntegral(uint32_t point) const override;
  enSignalTypes inline GetSignalType() const override {
    return SIG_GEN_TYPE_SINUS;
  }
};

/**
  * @brief  Класс прямоугольного сигнала
  */
class Square : public Signal {
 public:
  Square(uint32_t sample_timer_period);
  FP_TYPE GetValue(uint32_t point) const override;
  std::pair<FP_TYPE, FP_TYPE> FreqMod(uint32_t point, Signal& fmod) const override;
  FP_TYPE GetIntegral(uint32_t point) const override;
  enSignalTypes inline GetSignalType() const override {
    return SIG_GEN_TYPE_SQUARE;
  }

 private:
  FP_TYPE square(FP_TYPE t) const;
};

/**
  * @brief  Класс треугольного сигнала
  */
class Triangle : public Signal {
 public:
  Triangle(uint32_t sample_timer_period);
  FP_TYPE GetValue(uint32_t point) const override;
  std::pair<FP_TYPE, FP_TYPE> FreqMod(uint32_t point, Signal& fmod) const override;
  FP_TYPE GetIntegral(uint32_t point) const override;
  enSignalTypes inline GetSignalType() const override
	{
    return SIG_GEN_TYPE_TRIANGLE;
  }

 private:
  FP_TYPE triangle(FP_TYPE t) const;
};

/**
  * @brief  Класс пилообразного сигнала
  */
class Saw : public Signal {
 public:
  Saw(uint32_t sample_timer_period);
  FP_TYPE GetValue(uint32_t point) const override;
  std::pair<FP_TYPE, FP_TYPE> FreqMod(uint32_t point, Signal& fmod) const override;
  FP_TYPE GetIntegral(uint32_t point) const override;
  enSignalTypes inline GetSignalType() const override {
    return SIG_GEN_TYPE_SAW;
  }

 private:
  FP_TYPE sawtooth(FP_TYPE t) const;
};

// class Signal --------------------------------------------------------------

inline Signal::Signal(uint32_t sample_timer_period)
  : sample_rate_((FP_TYPE) sample_timer_period)
{}

inline void Signal::SetParam(uint8_t param, FP_TYPE value) {
  if (param == (uint8_t)SIG_GEN_PARAM_AMP) {
    SetAmp(value);
  } else if (param == (uint8_t)SIG_GEN_PARAM_FREQ) {
    SetFreq(value);
  }
}

inline Signal& Signal::SetAmp(FP_TYPE amp) {
  amp_ = amp;
  return *this;
}

inline Signal& Signal::SetFreq(FP_TYPE freq) {
  freq_ = freq;
  period_ = 1.0f / freq_;
  return *this;
}

inline Signal& Signal::SetFmodDepth(FP_TYPE mod_depth_percent) {
  fmod_depth_percent_ = (uint8_t) mod_depth_percent;
  return *this;
}

inline FP_TYPE Signal::GetAmp() const {
  return amp_;
}

inline FP_TYPE Signal::GetFreq() const {
  return freq_;
}

inline FP_TYPE Signal::GetModDepth() const {
  return fmod_depth_percent_;
}

inline std::unique_ptr<Signal> Signal::Create(enSignalTypes sig_type) {
  if (sig_type == SIG_GEN_TYPE_NONE) {
    return nullptr;
  } else if (sig_type == SIG_GEN_TYPE_SINUS) {
    return std::make_unique<Sinus>(sample_rate_);
  } else if (sig_type == SIG_GEN_TYPE_SQUARE) {
    return std::make_unique<Square>(sample_rate_);
  } else if (sig_type == SIG_GEN_TYPE_TRIANGLE) {
    return std::make_unique<Triangle>(sample_rate_);
  } else if (sig_type == SIG_GEN_TYPE_SAW) {
    return std::make_unique<Saw>(sample_rate_);
  }
  return nullptr;
}

// class Sinus ---------------------------------------------------------------

inline Sinus::Sinus(uint32_t sample_timer_period)
  : Signal(sample_timer_period)
{}

inline FP_TYPE Sinus::GetValue(uint32_t point) const {
  FP_TYPE t = (FP_TYPE)point / sample_rate_;
  return amp_ * std::sinf(2.f * pi * freq_ * t);
}

inline std::pair<FP_TYPE, FP_TYPE> Sinus::FreqMod(uint32_t point, Signal& fmod) const {
  FP_TYPE t = (FP_TYPE)point / sample_rate_;
  FP_TYPE beta = fmod_depth_percent_ / 100.f; // индекс модуляции
  FP_TYPE phase = 2.f * pi * freq_ * t + beta * fmod.GetIntegral(point);
  FP_TYPE result_amp = amp_ * std::sinf(phase);
  FP_TYPE instant_freq = freq_ + beta * fmod.GetFreq() * fmod.GetValue(point);
  return std::make_pair(result_amp, instant_freq);
}

inline FP_TYPE Sinus::GetIntegral(uint32_t point) const {
  FP_TYPE t = (FP_TYPE)point / sample_rate_;
  return (-1.0f) * std::cosf(2.0f * pi * freq_ * t);
}

// class Square ---------------------------------------------------------------

inline Square::Square(uint32_t sample_timer_period)
  : Signal(sample_timer_period)
{}

inline FP_TYPE Square::GetValue(uint32_t point) const {
  int sample = (int)point % (int)(period_ * (FP_TYPE)sample_rate_);
  FP_TYPE t = (FP_TYPE)sample / (FP_TYPE)sample_rate_;
  return square(t);
}

inline std::pair<FP_TYPE, FP_TYPE> Square::FreqMod(uint32_t point, Signal& /*fmod*/) const {
  return std::make_pair(0, 0);
}

inline FP_TYPE Square::GetIntegral(uint32_t point) const {
  int sample = (int)point % (int)(period_ * sample_rate_);
  FP_TYPE t = (FP_TYPE)sample / sample_rate_;
  if (t < period_ / 2.0f) {
    return t;
  }
  return -t;
}

inline FP_TYPE Square::square(FP_TYPE t) const {
  if (t < period_ / 2.0f) {
    return amp_;
  }
  return -amp_;
}

// class Triangle -------------------------------------------------------------

inline Triangle::Triangle(uint32_t sample_timer_period)
  : Signal(sample_timer_period)
{}

inline FP_TYPE Triangle::GetValue(uint32_t point) const {
  int sample = (int)point % (int)(period_ * sample_rate_);
  FP_TYPE t = (FP_TYPE)sample / sample_rate_;
  return amp_ * triangle(t);
}

inline std::pair<FP_TYPE, FP_TYPE> Triangle::FreqMod(uint32_t point, Signal& /*fmod*/) const {
  return std::make_pair(0, 0);
}

inline FP_TYPE Triangle::GetIntegral(uint32_t point) const {
  int sample = (int)point % (int)(period_ * sample_rate_);
  FP_TYPE t = (FP_TYPE)sample / sample_rate_;

  if (t < period_ / 4.0f) {
    return t * (2.0f * t / period_);
  } else if (t >= period_ / 4.0f && t < period_ * 3.0f / 4.0f) {
    return t * (2.0f - 2.0f * t / period_);
  } else {
    return t * (2.0f * t / period_ - 4.0f);
  }
}

inline FP_TYPE Triangle::triangle(FP_TYPE t) const {
  if (t < period_ / 4.0f) {
    return 4.0f * t / period_;
  } else if (t >= period_ / 4.0f && t < period_ * 3.0f / 4.0f) {
		return 2.0f - 4.0f * t / period_;
  } else {
    return 4.0f * t / period_ - 4.0f;
  }
}

// class Saw ---------------------------------------------------------------

inline Saw::Saw(uint32_t sample_timer_period)
  : Signal(sample_timer_period)
{}

inline FP_TYPE Saw::GetValue(uint32_t point) const {
  int sample = (int)point % (int)(period_ * sample_rate_);
  FP_TYPE t = (FP_TYPE)sample / sample_rate_;
  return amp_ * sawtooth(t);
}

inline std::pair<FP_TYPE, FP_TYPE> Saw::FreqMod(uint32_t point, Signal& /*fmod*/) const {
  return std::make_pair(0, 0);
}

inline FP_TYPE Saw::GetIntegral(uint32_t point) const {
  int sample = (int)point % (int)(period_ * sample_rate_);
  FP_TYPE t = (FP_TYPE)sample / sample_rate_;
  return t * (t - period_) / period_;
}

inline FP_TYPE Saw::sawtooth(FP_TYPE t) const {
  return 2.0f * t / period_ - 1.0f;
}

#endif // #ifndef _SIGNALS_H_