#ifndef _TIMER_FACILITY_H_
#define _TIMER_FACILITY_H_

#include "argument_user_types.h"
#include "callback.h"
#include "app/interrupt_manager.h"

#include "tim.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "timers.h"

#include <cassert>
#include <memory>

namespace tim {

// TimerInterface class =============================================
  
class TimerInterface {
 public:
  TimerInterface(TimerPeriod period_ms);
  virtual ~TimerInterface() {};
  virtual void Start() = 0;
  virtual void Stop() = 0;
  virtual void Reset() = 0;
  virtual void ChangePeriod(TimerPeriod period_ms) = 0;
  template<typename T>
  void SetExpireTimerHandler(T* owner, void (T::*func_ptr)());

 protected:
  TimerPeriod period_ms_;
  std::unique_ptr<_util::GenericCallback<>> expire_callback_;
};

template<typename T>
void TimerInterface::SetExpireTimerHandler(T* owner, void (T::*func_ptr)()) {
  expire_callback_ = std::make_unique<_util::Callback<T>>(owner, func_ptr);
}

inline TimerInterface::TimerInterface(TimerPeriod period_ms) :
  period_ms_(period_ms)
{}

// Software OS Timer class =============================================
  
class SW_FreeRTOS_Timer : public TimerInterface, public InterruptListener<TimerHandle_t> {
 public:  
  using TimerInterface::period_ms_;
  using TimerInterface::expire_callback_;
  
  SW_FreeRTOS_Timer(void* owner,
                    TimerPeriod period_ms,
                    enAutoreload auto_reload);
  ~SW_FreeRTOS_Timer() override;
  void Start() override;
  void Stop() override;
  void Reset() override;
  void ChangePeriod(TimerPeriod period_ms) override;
  TimerHandle_t GetHandler();
  bool ISR_Handler(TimerHandle_t timer) override;
  
 private:
  void InitFreeRTOS_Timer();
  
  void* owner_;
  TimerHandle_t tim_handle_;
  enAutoreload auto_reload_;
};

inline SW_FreeRTOS_Timer::SW_FreeRTOS_Timer(void* owner,
                                        TimerPeriod period_ms,
                                        enAutoreload auto_reload) :
      owner_(owner),
      TimerInterface(period_ms),
      auto_reload_(auto_reload) {
  InitFreeRTOS_Timer();
  os_tim_interrupt_manager.SetListener(this);
}

inline SW_FreeRTOS_Timer::~SW_FreeRTOS_Timer() {
  Stop();
  xTimerDelete(tim_handle_, osWaitForever);
}

inline void SW_FreeRTOS_Timer::Start() {
  xTimerStart(tim_handle_, osWaitForever);
}

inline void SW_FreeRTOS_Timer::Stop() {
  xTimerStop(tim_handle_, osWaitForever);
}

inline void SW_FreeRTOS_Timer::Reset() {
  xTimerReset(tim_handle_, osWaitForever);
}

inline void SW_FreeRTOS_Timer::ChangePeriod(TimerPeriod period_ms) {
  period_ms_ = period_ms;
  xTimerChangePeriod(tim_handle_, pdMS_TO_TICKS(period_ms_), osWaitForever);
}

inline TimerHandle_t SW_FreeRTOS_Timer::GetHandler() {
  return tim_handle_;
}

inline void SW_FreeRTOS_Timer::InitFreeRTOS_Timer() {
	tim_handle_ = xTimerCreate
					 ( /* Just a text name, not used by the RTOS
						 kernel. */
						 "Timer",
						 /* The timer period in ticks, must be
						 greater than 0. */
						 pdMS_TO_TICKS(period_ms_),
						 /* The timers will auto-reload themselves
						 when they expire. */
						 auto_reload_,
						 /* The ID is used to store a count of the
						 number of times the timer has expired, which
						 is initialised to 0. */
						 ( void * ) owner_,
						 /* Each timer calls the same expire_callback when
						 it expires. */
						 vTimerCallback
					 );

	if( tim_handle_ == NULL )
	{
		 /* The timer was not created. */
	}
}

inline bool SW_FreeRTOS_Timer::ISR_Handler(TimerHandle_t timer) {
  if(tim_handle_ == timer)
	{
    if(expire_callback_->isValid())
      expire_callback_->execute();
    return true;
  }
  return false;
}

// Hardware STM32 Timer class =============================================
  
class HW_STM32_BaseTimer : public TimerInterface, public InterruptListener<TIM_HandleTypeDef*> {
 public:
	using TimerInterface::period_ms_;
  using TimerInterface::expire_callback_;
  
  HW_STM32_BaseTimer(TIM_HandleTypeDef* hal_handler,
                     TimerPeriod period_ms);
  ~HW_STM32_BaseTimer();
  void Start() override;
  void Stop() override;
  void Reset() override;
  void ChangePeriod(TimerPeriod period_ms) override;
  TIM_HandleTypeDef* GetHandler();
  bool ISR_Handler(TIM_HandleTypeDef* timer) override;
  
 private:  
  TIM_HandleTypeDef* hal_handler_;
  int id_ = 0;
  size_t counter_;
};

inline HW_STM32_BaseTimer::HW_STM32_BaseTimer(TIM_HandleTypeDef* hal_handler,
                                          TimerPeriod period_ms) :
      hal_handler_(hal_handler),
      TimerInterface(period_ms),
      counter_(0) {
  hw_tim_interrupt_manager.SetListener(this);
  HAL_TIM_RegisterCallback(hal_handler_, HAL_TIM_PERIOD_ELAPSED_CB_ID, HW_TimBaseCallback);
}

inline HW_STM32_BaseTimer::~HW_STM32_BaseTimer() {
  Stop();
  hw_tim_interrupt_manager.RemoveListener(this);
}

inline void HW_STM32_BaseTimer::Start() {
  HAL_TIM_StateTypeDef tim_state = HAL_TIM_Base_GetState(hal_handler_);
  if(tim_state == HAL_TIM_STATE_READY)
	{
    HAL_TIM_Base_Start_IT(hal_handler_);
  }
  else if (tim_state == HAL_TIM_STATE_BUSY)
	{
  }
  else
	{
    assert(false);
  }
}

inline void HW_STM32_BaseTimer::Stop() {
  HAL_TIM_Base_Stop_IT(hal_handler_);
}

inline void HW_STM32_BaseTimer::Reset() {
  Stop();
  Start();
}

inline void HW_STM32_BaseTimer::ChangePeriod(TimerPeriod period_ms) {
  period_ms_ = period_ms;
}

inline TIM_HandleTypeDef* HW_STM32_BaseTimer::GetHandler() {
  return hal_handler_;
}

inline bool HW_STM32_BaseTimer::ISR_Handler(TIM_HandleTypeDef* timer) {
  if(hal_handler_ == timer)
	{
    ++counter_;
    if(counter_ >= period_ms_ && expire_callback_->isValid())
		{
      expire_callback_->execute();
      counter_ = 0;
    }
    Start();
    return true;
  }
  return false;
}

} // namespace tim

#endif // _TIMER_FACILITY_H_