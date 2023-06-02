#ifndef _PWM_TIM_CONTROLLER_H_
#define _PWM_TIM_CONTROLLER_H_

#include "tim.h"

#include <memory>
#include <vector>
#include <initializer_list>
#include <utility>

namespace pwm {
  
struct InitSettings {
  TIM_HandleTypeDef* timer;
  uint32_t channel;
};

class BaseController {
  enum State {
    STOPPED = 0,
    STARTED_VALUE_IS_ZERO,
    STARTED_VALUE_NON_ZERO,
    STATES_NUM
  };
  
public:
  BaseController(std::initializer_list<InitSettings> settings)
    : settings_(settings) {
    }
    virtual ~BaseController() {
      Stop();
    }
    
    virtual void Start() {
      for(const auto& settings : settings_) {
        HAL_TIM_PWM_Start(settings.timer, settings.channel);
        __HAL_TIM_SET_COMPARE(settings.timer, settings.channel, 0);
      }
      state_ = STARTED_VALUE_IS_ZERO;
    }
    
    void Stop() {
      for(const auto& settings : settings_) {
        HAL_TIM_PWM_Stop(settings.timer, settings.channel);
        HAL_TIM_PWM_Stop_IT(settings.timer, settings.channel);
        HAL_TIM_PWM_Stop_DMA(settings.timer, settings.channel);
      }
      state_ = STOPPED;
    }
    
    const auto& GetSettings() const { return settings_; }
    
    void SetPwmAllChannels(uint32_t value) {
      for(const auto& settings : settings_) {
        __HAL_TIM_SET_COMPARE(settings.timer, settings.channel, value);
      }
      state_ = (value > 0 ?  STARTED_VALUE_NON_ZERO : STARTED_VALUE_IS_ZERO);
    }
    
    void SetPwmSingleChannel(uint32_t channel, uint32_t value) {
      __HAL_TIM_SET_COMPARE(settings_[channel].timer,
                            settings_[channel].channel,
                            value);
      
      state_ = (value > 0 ?  STARTED_VALUE_NON_ZERO : STARTED_VALUE_IS_ZERO);
    }
    
protected:
  std::vector<InitSettings> settings_;
  State state_ = STOPPED;
};

//class IT_Controller ----------------------------------------------------

class IT_Controller : public BaseController {    
public:
  IT_Controller(std::initializer_list<InitSettings> settings)
    : BaseController(settings)
    {}
    
    void Start() override {
      for (const auto& settings : settings_) {
        HAL_TIM_PWM_Start_IT(settings.timer, settings.channel);
        __HAL_TIM_SET_COMPARE(settings.timer, settings.channel, 0);
      }
    }
};
  
} // namespace pwm

#endif // #ifndef _PWM_TIM_CONTROLLER_H_