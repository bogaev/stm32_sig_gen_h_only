#ifndef _ADC_CONTROLLER_H_
#define _ADC_CONTROLLER_H_

#include "adc.h"
#include "app/interrupt_manager.h"
#include "utility/callback.h"

#include <vector>
#include <numeric>
#include <cassert>
#include <memory>

namespace adc {

struct InitSettings {
  ADC_HandleTypeDef* hadc;
  size_t buffer_size;
};

// ADC BaseController class =============================================

class BaseController : public InterruptListener<ADC_HandleTypeDef*> {
public:
  BaseController(InitSettings settings)
    : hadc_(settings.hadc) {

    adc_interrupt_manager.SetListener(this);
  }

  virtual ~BaseController() {
    Stop();
  }

  virtual void Start() = 0;

  void Stop() {
    HAL_ADC_Stop(hadc_);
    HAL_ADC_Stop_IT(hadc_);
    HAL_ADC_Stop_DMA(hadc_);
  }

  template<typename T>
  void SetAdcConversationEndHandler(T* owner, void (T::*func_ptr)());
  bool ISR_Handler(ADC_HandleTypeDef* hadc) override {
    if(hadc_ == hadc) {
      if(measure_end_callback_->isValid())
        measure_end_callback_->execute();
      return true;
    }
    return false;
  }

protected:
  std::unique_ptr<_util::GenericCallback<>> measure_end_callback_;
  ADC_HandleTypeDef* hadc_ = nullptr;
};

template<typename T>
void BaseController::SetAdcConversationEndHandler(T* owner, void (T::*func_ptr)()) {
  measure_end_callback_ = std::make_unique<_util::Callback<T>>(owner, func_ptr);
}

// ADC IT_Controller class =============================================

class IT_Controller : public BaseController {
public:
  using BaseController::hadc_;

  IT_Controller(InitSettings settings) :
    BaseController(settings)
  {}

  void Start() override {
    HAL_ADC_Start_IT(hadc_);
  }

  uint16_t GetValue() {
    return HAL_ADC_GetValue(hadc_);
  }
};

// ADC DMAController =============================================

class DMAController : public BaseController {
public:
  using BaseController::hadc_;

  DMAController(InitSettings settings)
    : BaseController(settings),
    buffer_(settings.buffer_size, 0) {
  }

  void Start() override {
    HAL_ADC_Start_DMA(hadc_, (uint32_t*) buffer_.data(), buffer_.size());
  }

  uint16_t GetValue() {
    return (uint16_t)((double)std::accumulate(buffer_.cbegin(), buffer_.cend(), 0)
            / (double)buffer_.size());
  }

private:
  std::vector<uint16_t> buffer_;
};

} // namespace adc

#endif // #ifndef _ADC_CONTROLLER_H_