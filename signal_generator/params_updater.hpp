/**
  ******************************************************************************
  * @file    params_updater.h
  * @author  bogaev.s@gmail.com
  * @brief   Класс для обновления параметров сигнала
  ******************************************************************************
  */
    
#ifndef _PARAMS_UPDATER_H_
#define _PARAMS_UPDATER_H_

#include "utility\observer.hpp"
#include "common_\common.h"
#include "common_\signal_typedefs.h"

#include <initializer_list>


namespace miostim {

class SignalUpdater : public _pattern::Publisher<tdSignalParams> {
 public:
   
  SignalUpdater(std::initializer_list<_pattern::Observer<tdSignalParams>*> observers) {
   for (auto obs : observers) {
     RegisterObserver(obs);
   }
  }
   
  void UpdateParams(tdSignalParams params) {
    Notify(std::move(params));
  }
  
 private:
};

}

#endif // #ifndef _PARAMS_UPDATER_H_