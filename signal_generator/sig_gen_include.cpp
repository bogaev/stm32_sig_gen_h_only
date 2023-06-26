#include "signal_generator_cpp_interface.h"

SIG_GEN_StatusTypeDef SIG_GEN_Init(SIG_GEN_HandleTypeDef* sg_handle) {   
  return SignalGenerator::GetInstance().AddPwm(sg_handle);
}

SIG_GEN_StatusTypeDef SIG_GEN_Start(SIG_GEN_HandleTypeDef* sg_handle) {   
  return SignalGenerator::GetInstance().Start(sg_handle);
}

SIG_GEN_StatusTypeDef SIG_GEN_Deinit(SIG_GEN_HandleTypeDef* sg_handle) {   
  return SignalGenerator::GetInstance().DeletePwm(sg_handle);
}

SIG_GEN_StatusTypeDef SIG_GEN_SetSignalType(SIG_GEN_HandleTypeDef* sg_handle, enSignals signal, enSignalTypes type) {
  return SignalGenerator::GetInstance().SetSignalType(sg_handle, signal, type);
}

SIG_GEN_StatusTypeDef SIG_GEN_SetCarrierAmp(SIG_GEN_HandleTypeDef* sg_handle, FP_TYPE value) {
  return SignalGenerator::GetInstance().SetCarrierAmp(sg_handle, value);
}

SIG_GEN_StatusTypeDef SIG_GEN_SetFreq(SIG_GEN_HandleTypeDef* sg_handle, enSignals signal, FP_TYPE value) {
  return SignalGenerator::GetInstance().SetFreq(sg_handle, signal, value);
}

SIG_GEN_StatusTypeDef SIG_GEN_SetAmpModDepth(SIG_GEN_HandleTypeDef* sg_handle, uint8_t percent) {
  return SignalGenerator::GetInstance().SetAmpModDepth(sg_handle, percent);
}

SIG_GEN_StatusTypeDef SIG_GEN_SetFreqModSens(SIG_GEN_HandleTypeDef* sg_handle, uint8_t percent) {
  return SignalGenerator::GetInstance().SetFreqModSens(sg_handle, percent);
}

void SIG_GEN_Run(SIG_GEN_HandleTypeDef* sg_handle) {
  return SignalGenerator::GetInstance().Run(sg_handle);
}
