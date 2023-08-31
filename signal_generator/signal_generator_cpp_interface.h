#ifndef _SIGNAL_GENERATOR_H_
#define _SIGNAL_GENERATOR_H_

#include "app\sig_gen_config.h"
#include "app\common.h"
//#include "signal_generator\sig_gen.h"
#include "signal_generator\pwm_controller.h"
#include "utility\os_tasks_wrapper.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "main.h"
#include "cmsis_os.h"

#include <unordered_map>

class SignalGenerator {
 public:
  SignalGenerator(const SignalGenerator&) = delete;
  SignalGenerator& operator=(const SignalGenerator&) = delete;
  SignalGenerator(SignalGenerator&&) = delete;
  SignalGenerator& operator=(SignalGenerator&&) = delete;

  static SignalGenerator& GetInstance();

  // добавление нового генератора
  SIG_GEN_StatusTypeDef AddPwm(SIG_GEN_HandleTypeDef* sg_handle);
  SIG_GEN_StatusTypeDef Start(SIG_GEN_HandleTypeDef* sg_handle);
  SIG_GEN_StatusTypeDef Stop(SIG_GEN_HandleTypeDef* sg_handle);
  SIG_GEN_StatusTypeDef Resume(SIG_GEN_HandleTypeDef* sg_handle);
  SIG_GEN_StatusTypeDef Pause(SIG_GEN_HandleTypeDef* sg_handle);
  // удаление генератора
  SIG_GEN_StatusTypeDef DeletePwm(SIG_GEN_HandleTypeDef* sg_handle);
  // изменение типа сигнала
  SIG_GEN_StatusTypeDef SetSignalType(SIG_GEN_HandleTypeDef* sg_handle, enSignals signal, enSignalTypes type);
  // изменение значения амплитуды несущего сигнала
  SIG_GEN_StatusTypeDef SetCarrierAmp(SIG_GEN_HandleTypeDef* sg_handle, FP_TYPE value);
  // изменение частоты сигнала
  SIG_GEN_StatusTypeDef SetFreq(SIG_GEN_HandleTypeDef* sg_handle, enSignals signal, FP_TYPE value);
  // изменение глубины амплитудной модуляции
  SIG_GEN_StatusTypeDef SetAmpModDepth(SIG_GEN_HandleTypeDef* sg_handle, uint8_t percent);
  // установка коэффициента чувствительности частотной модуляции
  SIG_GEN_StatusTypeDef SetFreqModSens(SIG_GEN_HandleTypeDef* sg_handle, uint8_t percent);
  SIG_GEN_StatusTypeDef SetSignal(SIG_GEN_HandleTypeDef* sg_handle, uint8_t signal, uint8_t param, FP_TYPE value);
  SIG_GEN_StatusTypeDef CommitChanges(SIG_GEN_HandleTypeDef* sg_handle);
  void Run(SIG_GEN_HandleTypeDef* sg_handle); // TODO move to inner methods

 private:
  SignalGenerator() = default;

  SIG_GEN_StatusTypeDef CheckCorrStruct(SIG_GEN_RangeCoeff* array, uint32_t size);

#if DMA_GEN_TOTAL_NUM > 0 && DMA_BUF_SIZE > 0
  inline static BUF_DATA_TYPE dma_data_buf[DMA_GEN_TOTAL_NUM * DMA_BUF_SIZE] = {0};  // буфер данных DMA
#endif
#if IT_GEN_TOTAL_NUM > 0 && IT_BUF_SIZE > 0
  inline static IT_BUF_DATA_TYPE it_data_buf[IT_GEN_TOTAL_NUM * IT_BUF_SIZE] = {0}; // буфер данных IT
#endif

  std::unordered_map<SIG_GEN_HandleTypeDef*, PwmController*> pwms_;
  uint8_t it_gen_count_ = 0;
  uint8_t dma_gen_count_ = 0;
};

inline SignalGenerator& SignalGenerator::GetInstance() {
  static SignalGenerator sig_gen;
  return sig_gen;
}

inline SIG_GEN_StatusTypeDef SignalGenerator::CheckCorrStruct(SIG_GEN_RangeCoeff* array, uint32_t size) {
  if (array[0].from != 0) {
    return SIG_GEN_ERROR_INCORRECT_BOUNDS;
  }

  for (uint32_t i = 0; i < size-1; ++i) {
    FP_TYPE cur_end = array[i].to;
    FP_TYPE next_start = array[i+1].from;
    if (cur_end != next_start) {
      return SIG_GEN_ERROR_INCORRECT_BOUNDS;
    }
  }

  return SIG_GEN_OK;
}

inline SIG_GEN_StatusTypeDef SignalGenerator::Start(SIG_GEN_HandleTypeDef* sg_handle) {
  pwms_.at(sg_handle)->Start();
  return SIG_GEN_OK;
}

inline SIG_GEN_StatusTypeDef SignalGenerator::Stop(SIG_GEN_HandleTypeDef* sg_handle) {
  pwms_.at(sg_handle)->Stop();
  return SIG_GEN_OK;
}

inline SIG_GEN_StatusTypeDef SignalGenerator::Resume(SIG_GEN_HandleTypeDef* sg_handle) {
  pwms_.at(sg_handle)->Resume();
  return SIG_GEN_OK;
}

inline SIG_GEN_StatusTypeDef SignalGenerator::Pause(SIG_GEN_HandleTypeDef* sg_handle) {
  pwms_.at(sg_handle)->Pause();
  return SIG_GEN_OK;
}

inline SIG_GEN_StatusTypeDef SignalGenerator::AddPwm(SIG_GEN_HandleTypeDef* sg_handle) {
  if (pwms_.count(sg_handle)) {
    SIG_GEN_Deinit(sg_handle);
  }

  if (sg_handle->pwm_timer == 0) {
    return SIG_GEN_ERROR_PWM_TIMER_NOT_SET;
  }

  //  FP_TYPE sample_rate =
  //    sg_handle->coeffs ?
  //      sg_handle->coeffs->freq_array[sg_handle->coeffs->freq_array_size-1].to * 2.
  //        : 1000.;

  FP_TYPE sample_rate = SAMPLE_RATE;

  SignalModulator sig_mod((uint32_t)sample_rate);

  pwm_gen::PwmGenerator::Settings duty_cycle_settings;
  duty_cycle_settings.min_percent = (FP_TYPE)sg_handle->min_duty_cycle_percent;
  duty_cycle_settings.max_percent = (FP_TYPE)sg_handle->max_duty_cycle_percent;
  duty_cycle_settings.timer_period = (FP_TYPE)sg_handle->pwm_timer->Init.Period;

  pwm_gen::SignalStabilizer::Settings stabilizer_settings = {0};
  if (sg_handle->coeffs != 0) {
    {
      auto err = CheckCorrStruct(sg_handle->coeffs->amp_array,
                                 sg_handle->coeffs->amp_array_size);
      if (err) {
        return err;
      }
    }
    {
      auto err = CheckCorrStruct(sg_handle->coeffs->freq_array,
                                 sg_handle->coeffs->freq_array_size);
      if (err) {
        return err;
      }
    }
    stabilizer_settings.coeffs = sg_handle->coeffs;
  }

  pwm_gen::PwmGenerator pwm_gen_(std::move(sig_mod),
				 duty_cycle_settings,
				 stabilizer_settings,
				 sg_handle->dead_time_th_percent);

#if IT_GEN_TOTAL_NUM > 0 && IT_BUF_SIZE > 0
  if (sg_handle->pwm_mode == SIG_GEN_IT_MODE) {
    int next_buf_shift = it_gen_count_ * IT_BUF_SIZE;
    pwms_[sg_handle] = new IT_PwmController(sg_handle->pwm_timer,
                                           {sg_handle->channels[0],
                                            sg_handle->channels[1]},
                                            std::move(pwm_gen_),
                                            sg_handle->sample_timer,
                                            it_data_buf + next_buf_shift,
                                            IT_BUF_SIZE);
    ++it_gen_count_;
  }
#endif

#if DMA_GEN_TOTAL_NUM > 0 && DMA_BUF_SIZE > 0
  if (sg_handle->pwm_mode == SIG_GEN_DMA_MODE) {
    int next_buf_shift = dma_gen_count_ * DMA_BUF_SIZE;
    pwms_[sg_handle] = new DMA_PwmController(sg_handle->pwm_timer,
                                            {sg_handle->channels[0],
                                             sg_handle->channels[1]},
                                             std::move(pwm_gen_),
                                             USE_DOUBLE_BUFFER,
                                             dma_data_buf + next_buf_shift,
                                             DMA_BUF_SIZE);
    ++dma_gen_count_;
  }
#endif

//  pwms_[sg_handle]->Start();

  return SIG_GEN_OK;
}

inline SIG_GEN_StatusTypeDef SignalGenerator::DeletePwm(SIG_GEN_HandleTypeDef* sg_handle) {
  pwms_.at(sg_handle)->~PwmController();
  pwms_.erase(sg_handle);
  return SIG_GEN_OK;
}

inline SIG_GEN_StatusTypeDef SignalGenerator::SetSignalType(SIG_GEN_HandleTypeDef* sg_handle, enSignals signal, enSignalTypes type) {
  if (!pwms_.count(sg_handle)) {
    return SIG_GEN_ERROR_PWM_NOT_INITED;
  }
  pwms_.at(sg_handle)->SetSignal(signal, SIG_GEN_PARAM_SIGNAL_TYPE, type);
  return SIG_GEN_OK;
}

inline SIG_GEN_StatusTypeDef SignalGenerator::SetCarrierAmp(SIG_GEN_HandleTypeDef* sg_handle, FP_TYPE value) {
  if (!pwms_.count(sg_handle)) {
    return SIG_GEN_ERROR_PWM_NOT_INITED;
  }

  if (value > sg_handle->coeffs->amp_array[sg_handle->coeffs->amp_array_size-1].to) {
    return SIG_GEN_ERROR_AMPLITUDE_VALUE_ABOVE_MAX;
  }

  pwms_.at(sg_handle)->SetSignal(SIG_GEN_CARRIER, SIG_GEN_PARAM_AMP, value);
  return SIG_GEN_OK;
}

inline SIG_GEN_StatusTypeDef SignalGenerator::SetFreq(SIG_GEN_HandleTypeDef* sg_handle, enSignals signal, FP_TYPE value) {
//  FP_TYPE sample_rate =
//    sg_handle->coeffs ?
//      sg_handle->coeffs->freq_array[sg_handle->coeffs->freq_array_size-1].to * 2.
//        : 1000.;

  FP_TYPE sample_rate = SAMPLE_RATE;

  if (value >= sample_rate / 2.0f) {
    return SIG_GEN_ERROR_SAMPLE_RATE_LESS_THAN_FREQ;
  }

  if (!pwms_.count(sg_handle)) {
    return SIG_GEN_ERROR_PWM_NOT_INITED;
  }
  pwms_.at(sg_handle)->SetSignal(signal, SIG_GEN_PARAM_FREQ, value);
  return SIG_GEN_OK;
}

inline SIG_GEN_StatusTypeDef SignalGenerator::SetAmpModDepth(SIG_GEN_HandleTypeDef* sg_handle, uint8_t percent) {
  if (!pwms_.count(sg_handle)) {
    return SIG_GEN_ERROR_PWM_NOT_INITED;
  }
  pwms_.at(sg_handle)->SetSignal(SIG_GEN_AMP_MOD, SIG_GEN_PARAM_AMP_DEPTH, percent);
  return SIG_GEN_OK;
}

inline SIG_GEN_StatusTypeDef SignalGenerator::SetFreqModSens(SIG_GEN_HandleTypeDef* sg_handle, uint8_t percent) {
  if (!pwms_.count(sg_handle)) {
    return SIG_GEN_ERROR_PWM_NOT_INITED;
  }
  pwms_.at(sg_handle)->SetSignal(SIG_GEN_FREQ_MOD, SIG_GEN_PARAM_FREQ_DEPTH, percent);
  return SIG_GEN_OK;
}

inline SIG_GEN_StatusTypeDef SignalGenerator::SetSignal(SIG_GEN_HandleTypeDef* sg_handle, uint8_t signal, uint8_t param, FP_TYPE value) {
  if (!pwms_.count(sg_handle)) {
    return SIG_GEN_ERROR_PWM_NOT_INITED;
  }
  pwms_.at(sg_handle)->SetSignal(signal, param, value);
  return SIG_GEN_OK;
}

inline SIG_GEN_StatusTypeDef SignalGenerator::CommitChanges(SIG_GEN_HandleTypeDef* sg_handle) {
  if (!pwms_.count(sg_handle)) {
    return SIG_GEN_ERROR_PWM_NOT_INITED;
  }
  pwms_.at(sg_handle)->CommitChanges();
  return SIG_GEN_OK;
}

inline void SignalGenerator::Run(SIG_GEN_HandleTypeDef* sg_handle) {
  pwms_.at(sg_handle)->Run();
}

#endif // #ifndef _SIN_H_