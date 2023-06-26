#ifndef _SIG_GEN_WRAPPER_H_
#define _SIG_GEN_WRAPPER_H_

#include "app\sig_gen_config.h"
#include "app\signal_typedefs.h"

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

// Cтруктура инициализации генератора сигналов
typedef struct
{
  TIM_HandleTypeDef* pwm_timer;     // указатель на структуру ШИМ-генератора HAL
  TIM_HandleTypeDef* sample_timer;  // указатель на структуру таймера HAL для семплирования сигнала
  SIG_GEN_ModeTypeDef pwm_mode;     // режим работы таймера IT / DMA
  uint32_t channels[2];             // номера каналов для генерации сигнала (положительная и отрицательная полуволна)
  uint8_t min_duty_cycle_percent;   // минимальная величина коэффициента заполнения в процентах
  uint8_t max_duty_cycle_percent;   // максимальная величина коэффициента заполнения в процентах
  uint8_t dead_time_th_percent;     // dead time в процентах от периода сигнала
  SIG_GEN_CoeffsInitStruct* coeffs; // ссылка на структуру коэффициентов для корректировки амплитуды
} SIG_GEN_HandleTypeDef;

SIG_GEN_StatusTypeDef SIG_GEN_Init(SIG_GEN_HandleTypeDef* sg_handle);
SIG_GEN_StatusTypeDef SIG_GEN_Start(SIG_GEN_HandleTypeDef* sg_handle);
SIG_GEN_StatusTypeDef SIG_GEN_Deinit(SIG_GEN_HandleTypeDef* sg_handle);
SIG_GEN_StatusTypeDef SIG_GEN_SetSignalType(SIG_GEN_HandleTypeDef* sg_handle, enSignals signal, enSignalTypes type);
SIG_GEN_StatusTypeDef SIG_GEN_SetCarrierAmp(SIG_GEN_HandleTypeDef* sg_handle, FP_TYPE value);
SIG_GEN_StatusTypeDef SIG_GEN_SetFreq(SIG_GEN_HandleTypeDef* sg_handle, enSignals signal, FP_TYPE value);
SIG_GEN_StatusTypeDef SIG_GEN_SetAmpModDepth(SIG_GEN_HandleTypeDef* sg_handle, uint8_t percent);
SIG_GEN_StatusTypeDef SIG_GEN_SetFreqModSens(SIG_GEN_HandleTypeDef* sg_handle, uint8_t percent);
void SIG_GEN_Run(SIG_GEN_HandleTypeDef* sg_handle);

#ifdef __cplusplus
}
#endif

#endif // #ifndef _SIG_GEN_WRAPPER_H_