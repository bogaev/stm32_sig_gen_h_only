#ifndef _STM32_BASE_TIMER_CONTROLLER_H_
#define _STM32_BASE_TIMER_CONTROLLER_H_

#include "tim.h"

#include "utility\callback.hpp"
#include "utility\argument_user_types.hpp"

#include <memory>
#include <vector>
#include <initializer_list>
#include <utility>
#include <cassert>

namespace stm_timer {
  
struct InitSettings {
  TIM_HandleTypeDef* timer;
  uint32_t channel;
  TimerPeriod period_ms;
};

template<typename T>
class Controller {
  using _Callback = _util::Callback<T>;
  enum State {
    STOPPED = 0,
    STARTED_VALUE_IS_ZERO,
    STARTED_VALUE_NON_ZERO,
    STATES_NUM
  };

public:
  Controller(InitSettings settings, _Callback callback);
  virtual ~Controller();
  
  virtual void Start();
  void Stop();
  void SetCompare(uint32_t value);
  bool TimerHandler(TIM_HandleTypeDef* timer);
  const auto& GetSettings() const {
    return settings_;
  }

protected:
  InitSettings settings_;
  _Callback callback_;
  size_t counter_ = 0;
  State state_ = STOPPED;
};

template<typename T>
Controller<T>::Controller(InitSettings settings, _Callback callback)
  : settings_(settings)
  , callback_(callback)
{
}

template<typename T>
Controller<T>::~Controller() {
  Stop();
}

template<typename T>
void Controller<T>::Start() {
  HAL_TIM_StateTypeDef tim_state = HAL_TIM_Base_GetState(settings_.timer);
  if(tim_state == HAL_TIM_STATE_READY) {
    HAL_TIM_Base_Start(settings_.timer);
    __HAL_TIM_SET_COMPARE(settings_.timer, settings_.channel, 0);
    state_ = STARTED_VALUE_IS_ZERO;
  }
  else if (tim_state == HAL_TIM_STATE_BUSY) {
  }
  else {
    assert(false);
  }
}

template<typename T>
void Controller<T>::Stop() {
  HAL_TIM_Base_Stop(settings_.timer);
  HAL_TIM_Base_Stop_IT(settings_.timer);
  HAL_TIM_Base_Stop_DMA(settings_.timer);
  state_ = STOPPED;
}

template<typename T>
void Controller<T>::SetCompare(uint32_t value) {
  __HAL_TIM_SET_COMPARE(settings_.timer,
                        settings_.channel,
                        value);
  
  state_ = (value > 0 ?  STARTED_VALUE_NON_ZERO : STARTED_VALUE_IS_ZERO);
}

template<typename T>
bool Controller<T>::TimerHandler(TIM_HandleTypeDef* timer) {
  if(timer == settings_.timer) {
    ++counter_;
    if(counter_ >= settings_.period_ms) {
      callback_.execute();
    }
    return true;
  }
  return false;
}

//class IT_Controller ----------------------------------------------------

template<typename T>
class IT_Controller : public Controller<T> {    
public:
  IT_Controller(InitSettings settings);
  void Start() override;
};

template<typename T>
IT_Controller<T>::IT_Controller(InitSettings settings)
  : Controller<T>(settings)
{}

template<typename T>
void IT_Controller<T>::Start() {
  HAL_TIM_Base_Start_IT(Controller<T>::settings_.timer);
  __HAL_TIM_SET_COMPARE(Controller<T>::settings_.timer,
                        Controller<T>::settings_.channel, 0);
}

}

#endif // #ifndef _STM32_BASE_TIMER_CONTROLLER_H_