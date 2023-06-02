#ifndef _EXTI_CONTROLLER_H_
#define _EXTI_CONTROLLER_H_

#include "main.h"

#include "utility\callback.hpp"
#include "system\interrupt_manager.hpp"

#include <vector>
#include <cassert>

namespace exti {

template<typename T>
class Controller : InterruptListener<uint16_t> { 
public:
  Controller(uint16_t exti_pin);
  bool ISR_Handler(uint16_t GPIO_Pin) override;
  void SetCallback(T* owner, void (T::*func_ptr)());
  void SetCounter(size_t value);
  size_t GetCounter();
  
private:
  uint16_t exti_pin_ = 0;
  size_t counter_ = 0;
  _util::Callback<T> interrupt_callback_;
};

template<typename T>
Controller<T>::Controller(uint16_t exti_pin) :
  exti_pin_(exti_pin)
{
  exti_interrupt_manager.SetListener(this);
}

template<typename T>
bool Controller<T>::ISR_Handler(uint16_t GPIO_Pin) {
  if(exti_pin_ == GPIO_Pin) {
    ++counter_;
    if(interrupt_callback_.isValid()) {
      interrupt_callback_.execute();
    }
    return true;
  }
  return false;
}

template<typename T>
void Controller<T>::SetCounter(size_t value) {
  counter_ = value;
}

template<typename T>
size_t Controller<T>::GetCounter() {
  return counter_;
}

template<typename T>
void Controller<T>::SetCallback(T* owner, void (T::*func_ptr)()) {
  interrupt_callback_ = _util::Callback<T>(owner, func_ptr);
}

}

#endif // #ifndef _EXTI_CONTROLLER_H_