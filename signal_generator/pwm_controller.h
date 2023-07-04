/**
  ******************************************************************************
  * @file    pwm_generator.h
  * @author  bogaev.s@gmail.com
  * @brief   Файл определяет классы, управляющие генерацией ШИМ
  *          в различных режимах работы периферии STM32 (IT, DMA)
  ******************************************************************************
  */

#ifndef _PWM_CONTROLLER_H_
#define _PWM_CONTROLLER_H_

#include <memory>
#include <functional>
#include <unordered_map>
#include <vector>
//#include <assert.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "main.h"
#include "cmsis_os.h"
#include "tim.h"

#include "app\common.h"
#include "app\sig_gen_config.h"
#include "signal_generator\signals.h"
#include "signal_generator\pwm_generator.h"

#include "utility\os_tasks_wrapper.h"
#include "utility\callback.h"
#include "utility\observer.h"
#include "utility\cpu_utils.h"
#include "utility\shared_objects.h"

class RTOSTaskWrapper;

struct tdPwmChannels {
  uint32_t pos_halfwave_channel;
  uint32_t neg_halfwave_channel;
};

/**
  * @brief  Базовый класс для контроллера генерации ШИМ
  */
class PwmController : public _pattern::Observer<tdUartData> {
 public:
  PwmController(TIM_HandleTypeDef* timer,
                tdPwmChannels channels,
                pwm_gen::PwmGenerator generator);
  virtual ~PwmController();
  
  virtual void Start() = 0;
  virtual void Stop() = 0;
  virtual void Resume() {};
  virtual void Pause() {};
  virtual void Run() = 0;
  virtual void ISR_Handler() = 0;
  virtual void SetSignal(uint8_t signal, uint8_t param, FP_TYPE value);
  virtual bool IsPaused() const { return false; }
  
 protected:  
  static void PWM_PulseHalfFinishedCallback(TIM_HandleTypeDef *htim);
  static void PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
  static void BindTimerToInstance(TIM_HandleTypeDef* htim, PwmController* ptr);
  static std::vector<PwmController*> GetTimerInstance(TIM_HandleTypeDef* htim);
  static HAL_TIM_ActiveChannel GetActiveChannel(uint32_t tim_ch);
  void Update(tdUartData msg) override;
   
  inline static std::vector<std::vector<PwmController*>> htim_to_inst{10}; /// таблица хендлер таймера -> экземпляр класса
  TIM_HandleTypeDef* timer_ = nullptr; /// таймер ШИМ STM32
  tdPwmChannels channels_; /// каналы таймера ШИМ STM32
  pwm_gen::PwmGenerator generator_; /// генератор значений ШИМ
  bool is_active_ = false; /// текущий статус генерации сигнала
};

inline PwmController::PwmController(TIM_HandleTypeDef* timer,
                             tdPwmChannels channels,
                             pwm_gen::PwmGenerator generator)
			: timer_(timer)
			, generator_(std::move(generator))
			, channels_({channels.pos_halfwave_channel, channels.neg_halfwave_channel}) {
  SharedObjects::uart_received.RegisterObserver(this);
}

inline PwmController::~PwmController() {
  SharedObjects::uart_received.RemoveObserver(this);
}

inline void PwmController::SetSignal(uint8_t signal, uint8_t param, FP_TYPE value) {
  generator_.SetSignal(signal, param, value);
}

/**
  * @brief  Обработчик события обновления в классе "Издателя"
  */
inline void PwmController::Update(tdUartData msg) {
  generator_.SetSignal(msg.signal, msg.param, msg.value);
}

/**
  * @brief  Обработчик события "вывод половины значений буфера сигнала завершен"
  */
inline void PwmController::PWM_PulseHalfFinishedCallback(TIM_HandleTypeDef *htim) {
  auto handlers = GetTimerInstance(htim);
  for (auto& handler : handlers) {
    handler->ISR_Handler();
  }
}

/**
  * @brief  Обработчик события "вывод всех значений буфера сигнала завершен"
  */
inline void PwmController::PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
  auto handlers = GetTimerInstance(htim);
  for (auto& handler : handlers) {
    handler->ISR_Handler();
  }
}

/**
  * @brief  Привязка экземпляра класса контроллера (this) к хендлеру таймера (timer_)
  */
inline void PwmController::BindTimerToInstance(TIM_HandleTypeDef* htim, PwmController* ptr) {
  if (htim->Instance == TIM1) {
    htim_to_inst[0].push_back(ptr);
  } else if (htim->Instance == TIM2) {
    htim_to_inst[1].push_back(ptr);
  } else if (htim->Instance == TIM3) {
    htim_to_inst[2].push_back(ptr);
  } else if (htim->Instance == TIM4) {
    htim_to_inst[3].push_back(ptr);
  } else if (htim->Instance == TIM5) {
    htim_to_inst[4].push_back(ptr);
  } else if (htim->Instance == TIM8) {
    if (ptr->channels_.pos_halfwave_channel == TIM_CHANNEL_1) {
      htim_to_inst[5].push_back(ptr);
    } else {
      htim_to_inst[6].push_back(ptr);
    }
  } else if (htim->Instance == TIM12) {
    htim_to_inst[7].push_back(ptr);
  } else if (htim->Instance == TIM13) {
    htim_to_inst[8].push_back(ptr);
  } else if (htim->Instance == TIM14) {
    htim_to_inst[9].push_back(ptr);
  }
}

/**
  * @brief  Возвращает экземпляр класса (для обработчика прерывания)
  */
inline std::vector<PwmController*> PwmController::GetTimerInstance(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM1) {
    return htim_to_inst[0];
  } else if (htim->Instance == TIM2) {
    return htim_to_inst[1];
  } else if (htim->Instance == TIM3) {
    return htim_to_inst[2];
  } else if (htim->Instance == TIM4) {
    return htim_to_inst[3];
  } else if (htim->Instance == TIM5) {
    return htim_to_inst[4];
  } else if (htim->Instance == TIM8) {
    if (	 htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 
				|| htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
      return htim_to_inst[5];
    } else {
      return htim_to_inst[6];
    }
  } else if (htim->Instance == TIM12) {
    return htim_to_inst[7];
  } else if (htim->Instance == TIM13) {
    return htim_to_inst[8];
  } else if (htim->Instance == TIM14) {
    return htim_to_inst[9];
  }
  return htim_to_inst[0];
}

/**
  * @brief  Возвращает текущий активный канал таймера
  */
inline HAL_TIM_ActiveChannel PwmController::GetActiveChannel(uint32_t tim_ch) {
  if (tim_ch == TIM_CHANNEL_1) {
    return HAL_TIM_ACTIVE_CHANNEL_1;
  } else if (tim_ch == TIM_CHANNEL_2) {
    return HAL_TIM_ACTIVE_CHANNEL_2;
  } else if (tim_ch == TIM_CHANNEL_3) {
    return HAL_TIM_ACTIVE_CHANNEL_3;
  } else if (tim_ch == TIM_CHANNEL_4) {
    return HAL_TIM_ACTIVE_CHANNEL_4;
  }
  return HAL_TIM_ACTIVE_CHANNEL_1;
}

// class IT_PwmController ============================================================

/**
  * @brief  Класс контроллера генерации ШИМ в режиме прерывания (IT)
  */
class IT_PwmController : public PwmController {    
 public:
  IT_PwmController(TIM_HandleTypeDef* timer,
                   tdPwmChannels channels,
                   pwm_gen::PwmGenerator generator,
                   TIM_HandleTypeDef* sample_timer,
                   IT_BUF_DATA_TYPE* buf_ptr,
                   uint32_t buf_size);
  virtual ~IT_PwmController();
  
  void Start() override;
  void Stop() override;
  void Resume() override;
  void Pause() override;
  void Run() override;
  void SetSignal(uint8_t signal, uint8_t param, FP_TYPE value) override;
  bool IsPaused() const override;

 private:
  static void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
  void ISR_Handler() override;
  void TaskPwmStart(void *argument);
  void TaskBufUpd(void *argument); /// код задачи FreeRTOS обновления буфера
  void UpdateBuffer();

  TIM_HandleTypeDef* const sample_timer_; /// таймер семплирования сигнала
  IT_BUF_DATA_TYPE* const buf_ptr_; /// указатель на начало буфера значений сигнала
  const uint32_t buf_size_; /// размер буфера значений сигнала
  uint32_t index_ = 0; /// текущий индекс массива значений сигнала
  bool is_paused_ = false; /// текущий индекс массива значений сигнала
  osSemaphoreId_t pwm_start_;
  osSemaphoreId_t pwm_transfer_complete_;
  osSemaphoreId_t calc_upd_buf_sem_;
  std::unique_ptr<RTOSTaskWrapper> task_start_; /// обёртка задачи FreeRTOS
  std::unique_ptr<RTOSTaskWrapper> task_buf_upd_; /// обёртка задачи FreeRTOS
};

inline IT_PwmController::IT_PwmController(TIM_HandleTypeDef* timer,
					 tdPwmChannels channels,
					 pwm_gen::PwmGenerator generator,
					 TIM_HandleTypeDef* sample_timer,
					 IT_BUF_DATA_TYPE* buf_ptr,
					 uint32_t buf_size)
        : PwmController(timer, channels, std::move(generator))
        , sample_timer_(sample_timer)
        , buf_ptr_(buf_ptr)
        , buf_size_(buf_size)
        , pwm_start_(osSemaphoreNew(1U, 1U, NULL))
        , pwm_transfer_complete_(osSemaphoreNew(1U, 0U, NULL))
        , calc_upd_buf_sem_(osSemaphoreNew(1U, 0U, NULL)) {
  task_start_ = 
    std::make_unique<RTOSTaskWrapper>(
        "start", RTOS_TASK_STACK_SIZE, osPriorityHigh,
         std::function<void(void*)>(std::bind(&IT_PwmController::TaskPwmStart, this, std::placeholders::_1)));

  task_buf_upd_ = 
    std::make_unique<RTOSTaskWrapper>("buf_upd", RTOS_TASK_STACK_SIZE, osPriorityNormal,
                                      std::function<void(void*)>(std::bind(&IT_PwmController::TaskBufUpd, this, std::placeholders::_1)));
  
  HAL_TIM_RegisterCallback(sample_timer_, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_PeriodElapsedCallback);
  BindTimerToInstance(sample_timer_, this);
  task_start_->Create();
  task_buf_upd_->Create();
}

inline IT_PwmController::~IT_PwmController() {
  Stop();
  task_buf_upd_->Delete();
  task_start_->Delete();
}

inline void IT_PwmController::SetSignal(uint8_t signal, uint8_t param, FP_TYPE value) {
  PwmController::SetSignal(signal, param, value);
  osSemaphoreRelease(calc_upd_buf_sem_);
}

inline void IT_PwmController::Start() {
  HAL_TIM_Base_Start_IT(sample_timer_);
  HAL_TIM_PWM_Start_IT(timer_, channels_.pos_halfwave_channel);
  HAL_TIM_PWM_Start_IT(timer_, channels_.neg_halfwave_channel);
}

inline void IT_PwmController::Stop() {
  HAL_TIM_PWM_Stop_IT(timer_, channels_.neg_halfwave_channel);
  HAL_TIM_PWM_Stop_IT(timer_, channels_.pos_halfwave_channel);
  HAL_TIM_Base_Stop_IT(sample_timer_);
}

inline void IT_PwmController::Resume() {
  index_ = 0;
  is_paused_ = false;
}

inline void IT_PwmController::Pause() {
  is_paused_ = true;
  __HAL_TIM_SET_COMPARE(timer_, channels_.pos_halfwave_channel, 0);
  __HAL_TIM_SET_COMPARE(timer_, channels_.neg_halfwave_channel, 0);
}

inline bool IT_PwmController::IsPaused() const {
  return is_paused_;
}

inline void IT_PwmController::Run() {
  auto dc = buf_ptr_[index_++];
//  duty_cycle_monitor = dc;
  if (dc < 0) { // TODO убрать зависимость от знака числа?
    __HAL_TIM_SET_COMPARE(timer_, channels_.pos_halfwave_channel, 0);
    __HAL_TIM_SET_COMPARE(timer_, channels_.neg_halfwave_channel, -dc);
  } else {
    __HAL_TIM_SET_COMPARE(timer_, channels_.neg_halfwave_channel, 0);
    __HAL_TIM_SET_COMPARE(timer_, channels_.pos_halfwave_channel, dc);
  }
  index_ = (index_ >= buf_size_ ? 0 : index_);
}

inline void IT_PwmController::TaskPwmStart(void *argument) {	
  for (;;) {
    if (osSemaphoreAcquire( pwm_start_, osWaitForever ) == osOK) {
      HAL_TIM_Base_Start_IT(sample_timer_);
      HAL_TIM_PWM_Start_IT(timer_, channels_.pos_halfwave_channel);
      HAL_TIM_PWM_Start_IT(timer_, channels_.neg_halfwave_channel);
      osSemaphoreRelease(pwm_transfer_complete_);
    }
  }
}

/**
  * @brief  Метод, содержащий код задачи FreeRTOS которая обновляет буфер значений
  */
inline void IT_PwmController::TaskBufUpd(void *argument) {
  for(;;) {
    if (osSemaphoreAcquire(calc_upd_buf_sem_, osWaitForever) == osOK) {
      UpdateBuffer();
    }
  }
}

inline void IT_PwmController::ISR_Handler() {
  Run();
}

inline void IT_PwmController::TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  auto instances = GetTimerInstance(htim);
  for (auto* instance : instances) {
    if (!instance->IsPaused()) {
      instance->ISR_Handler();
    }
  }
}

inline void IT_PwmController::UpdateBuffer() {
//  Pause();
  for (size_t i = 0; i < buf_size_; ++i) {
    if (generator_.IsNegHalfwave()) {
      buf_ptr_[i] = (-1) * (IT_BUF_DATA_TYPE) generator_.GetValue();
    } else {
      buf_ptr_[i] = (IT_BUF_DATA_TYPE) generator_.GetValue();
    }
  }
//  Resume();
}

// class DMA_PwmController ============================================================

/**
  * @brief  Класс контроллера генерации ШИМ в режиме прямого доступа к памяти (DMA)
  */
class DMA_PwmController : public PwmController {		
 public:
  DMA_PwmController(TIM_HandleTypeDef* timer,
                    tdPwmChannels channels,
                    pwm_gen::PwmGenerator generator,
                    BufferModeTypeDef buf_mode,
                    BUF_DATA_TYPE* buf_ptr,
                    uint32_t buf_size);
  virtual ~DMA_PwmController();
  
  void Start() override;
  void StartBufferUpdate();
  void Stop() override;
  void Run() override;
  void SetSignal(uint8_t signal, uint8_t param, FP_TYPE value) override;
  void UpdateBufferSize(size_t new_size);
  template<typename T>
  void SetBufferReadyHandler(T* owner, void (T::*func_ptr)(PwmController* obj)) {
    buf_ready_callback_ = std::make_unique<_util::Callback<T, PwmController*>>(owner, func_ptr);
  }
  
 private:
//  static void DMAHalfFinishedCallback(DMA_HandleTypeDef *hdma);
//  static void DMAFinishedCallback(DMA_HandleTypeDef *hdma);
  void UpdateFrame(uint32_t size);
  void SwitchFrame();
  void SetAllDMAReady();
  void ISR_Handler() override;
  void TaskDmaStart(void *argument); /// код задачи FreeRTOS запуска таймера
  void TaskBufUpd(void *argument); /// код задачи FreeRTOS обновления буфера
  
  BufferModeTypeDef buf_mode_ = BUF_MODE_DOUBLE; /// режим буфера
  BUF_DATA_TYPE* const buf_ptr_; /// указатель на начало буфера значений ШИМ
  uint32_t buf_size_; /// размер буфера значений ШИМ

  uint32_t ch_buf_size_; /// размер буфера для одного канала
  BUF_DATA_TYPE* pos_ch_; /// буфер сигнала для положительной полуволны
  BUF_DATA_TYPE* neg_ch_; /// буфер сигнала для отрицательной полуволны

  osSemaphoreId_t calc_frame_sem_;
  osSemaphoreId_t calc_upd_buf_sem_;
  osSemaphoreId_t calc_buf_ready_sem_;
  osSemaphoreId_t pwm_start_;
  
//  bool is_buf_ready_ = false;
  std::unique_ptr<RTOSTaskWrapper> task_start_; /// обёртка задачи FreeRTOS
  std::unique_ptr<RTOSTaskWrapper> task_buf_upd_; /// обёртка задачи FreeRTOS
  std::unique_ptr<_util::GenericCallback<PwmController*>> buf_ready_callback_; /// callback сигнала об успешном обновлении буфера
};

inline DMA_PwmController::DMA_PwmController(TIM_HandleTypeDef* timer,
                                            tdPwmChannels channels,
                                            pwm_gen::PwmGenerator generator,
                                            BufferModeTypeDef buf_mode,
                                            BUF_DATA_TYPE* buf_ptr,
                                            uint32_t buf_size)
			: PwmController(timer, channels, std::move(generator))
			, buf_mode_(buf_mode)
			, buf_ptr_(buf_ptr)
			, buf_size_(buf_size)
			, ch_buf_size_(buf_size_/2)
			, pos_ch_(buf_ptr_)
			, neg_ch_(pos_ch_ + ch_buf_size_)
			, calc_frame_sem_(osSemaphoreNew(1, 0U, NULL))
			, calc_upd_buf_sem_(osSemaphoreNew(1, 0U, NULL))
			, calc_buf_ready_sem_(osSemaphoreNew(1, 0U, NULL))
			, pwm_start_(osSemaphoreNew(1, 0U, NULL)) {
//  assert(buf_ptr_);
//  assert(buf_size_);
//  assert(ch_buf_size_);
//  assert(pos_ch_);
//  assert(neg_ch_);
//  assert(calc_upd_buf_sem_);
//  assert(calc_buf_ready_sem_);
//  assert(pwm_start_);
  
  task_start_ = 
    std::make_unique<RTOSTaskWrapper>("start", RTOS_TASK_STACK_SIZE, osPriorityHigh,
                                      std::function<void(void*)>(std::bind(&DMA_PwmController::TaskDmaStart, this, std::placeholders::_1)));
    
  task_buf_upd_ = 
    std::make_unique<RTOSTaskWrapper>("buf_upd", RTOS_TASK_STACK_SIZE, osPriorityNormal,
                                      std::function<void(void*)>(std::bind(&DMA_PwmController::TaskBufUpd, this, std::placeholders::_1)));
  
  if (buf_mode_ != BUF_MODE_SINGLE) {
    HAL_TIM_RegisterCallback(timer_, HAL_TIM_PWM_PULSE_FINISHED_HALF_CB_ID, PWM_PulseHalfFinishedCallback);
    HAL_TIM_RegisterCallback(timer_, HAL_TIM_PWM_PULSE_FINISHED_CB_ID, PWM_PulseFinishedCallback);
  }
  //	HAL_DMA_RegisterCallback(timer_->hdma[TIM_DMA_ID_CC1], HAL_DMA_XFER_HALFCPLT_CB_ID, DMAHalfFinishedCallback);
  //	HAL_DMA_RegisterCallback(timer_->hdma[TIM_DMA_ID_CC1], HAL_DMA_XFER_CPLT_CB_ID, DMAFinishedCallback);
  
  BindTimerToInstance(timer_, this);
  task_start_->Create();
  task_buf_upd_->Create();
}

inline DMA_PwmController::~DMA_PwmController() {
  Stop();
  task_buf_upd_->Delete();
  task_start_->Delete();
}

inline void DMA_PwmController::SetSignal(uint8_t signal, uint8_t param, FP_TYPE value) {
  PwmController::SetSignal(signal, param, value);
  if (buf_mode_ == BUF_MODE_SINGLE) {
 		osSemaphoreRelease(calc_upd_buf_sem_);
  }
}

inline void DMA_PwmController::UpdateBufferSize(size_t new_size) {
  buf_size_ = new_size;
  ch_buf_size_ = buf_size_ / 2;
}

inline void DMA_PwmController::StartBufferUpdate() {
  osSemaphoreRelease(calc_upd_buf_sem_);
}

inline void DMA_PwmController::Start() {
  StartBufferUpdate();
	if (!buf_ready_callback_ || !buf_ready_callback_->isValid()) {
		Run();
	}
}

inline void DMA_PwmController::Stop() {
  HAL_TIM_PWM_Stop_DMA(timer_, channels_.pos_halfwave_channel);
  HAL_TIM_PWM_Stop_DMA(timer_, channels_.neg_halfwave_channel);
  SetAllDMAReady();
}

inline void DMA_PwmController::Run() {
  HAL_TIM_PWM_Start_DMA(timer_, channels_.pos_halfwave_channel, (uint32_t*)pos_ch_, ch_buf_size_);
  HAL_TIM_PWM_Start_DMA(timer_, channels_.neg_halfwave_channel, (uint32_t*)neg_ch_, ch_buf_size_);
}

/**
  * @brief  Метод, содержащий код задачи FreeRTOS которая запускает ШИМ
  */
inline void DMA_PwmController::TaskDmaStart(void *argument) {
  for(;;) {
    if ( osSemaphoreAcquire( pwm_start_, osWaitForever ) == osOK ) {
      HAL_TIM_PWM_Start_DMA(timer_, channels_.pos_halfwave_channel, (uint32_t*)pos_ch_, ch_buf_size_);
      HAL_TIM_PWM_Start_DMA(timer_, channels_.neg_halfwave_channel, (uint32_t*)neg_ch_, ch_buf_size_);
    }
  }
}

/**
  * @brief  Метод, содержащий код задачи FreeRTOS которая обновляет буфер значений
  */
inline void DMA_PwmController::TaskBufUpd(void *argument) {
  for(;;) {
    if (osSemaphoreAcquire(calc_upd_buf_sem_, osWaitForever) == osOK) {
      if (buf_mode_ == BUF_MODE_DOUBLE) {
        SwitchFrame();
        UpdateFrame(ch_buf_size_ / 2);
      } else {
        UpdateFrame(ch_buf_size_);
      }
			
      if (buf_ready_callback_ && buf_ready_callback_->isValid()) {
        buf_ready_callback_->execute(this);
      }
    }
  }
}

/**
  * @brief  Обработчик прерывания таймера
  */
inline void DMA_PwmController::ISR_Handler() {
  if (timer_->Channel == GetActiveChannel(channels_.pos_halfwave_channel)) {
//    SwitchFrame();
//    UpdateFrame(ch_buf_size_/2);
 		osSemaphoreRelease(calc_upd_buf_sem_);
  }
}

/**
  * @brief  Обновление буфера значений ШИМ
  */
inline void DMA_PwmController::UpdateFrame(uint32_t size) {
//	static BUF_DATA_TYPE dc = 0;
  for (size_t i = 0; i < size; ++i) {
//    StartCyclesCounter();
//    static BUF_DATA_TYPE cnt = 0;
//    cnt = (cnt > timer_->Init.Period ? 0 : cnt);
//    cnt += 1;
//    duty_cycle_monitor = cnt;
//    pos_ch_[i] = dc;
//    neg_ch_[i] = 0;
//    dc += 100;
//    BUF_DATA_TYPE dc = (BUF_DATA_TYPE) generator_.GetValue();
//    cycles_num_debug = GetCyclesCounter();
//    StopCyclesCounter();
    
    BUF_DATA_TYPE dc = (BUF_DATA_TYPE) generator_.GetValue();
    if (generator_.IsNegHalfwave()) {
      pos_ch_[i] = 0;
      neg_ch_[i] = dc;
    } else {
      pos_ch_[i] = dc;
      neg_ch_[i] = 0;
    }
		
//		pos_ch_[i] = 0;
//		neg_ch_[i] = dc;
//		dc += 100;
  }
//	dc = 0;
}

/**
  * @brief  Переключение адреса начала буфера (для двойной буферизации)
  */
inline void DMA_PwmController::SwitchFrame() {
  if (pos_ch_ == buf_ptr_) {
    pos_ch_ = pos_ch_ + ch_buf_size_ / 2;
    neg_ch_ = neg_ch_ + ch_buf_size_ / 2;
  } else {
    pos_ch_ = buf_ptr_;
    neg_ch_ = pos_ch_ + ch_buf_size_;
  }
}

/**
  * @brief  Сброс состояния DMA после остановки DMA
  */
inline void DMA_PwmController::SetAllDMAReady() {
  for (int i = 0; i < 7; ++i) {
    timer_->hdma[i]->State = HAL_DMA_STATE_READY;
    __HAL_UNLOCK(timer_->hdma[i]);	  
  }
}

#endif // #ifndef _PWM_CONTROLLER_H_