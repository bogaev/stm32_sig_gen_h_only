#ifndef _TREATMENT_PATTERNS_H_
#define _TREATMENT_PATTERNS_H_

#include "app/common.h"

#include "signal_generator/pwm_controller.h"

#include "utility/observer.h"
#include "utility/timer_facility.h"

#include <vector>
#include <array>
#include <vector>
#include <functional>

namespace miostim {

using namespace miostim::mod_values;

class TreatPattern : public _pattern::Observer<tdUartData> {
public:
  TreatPattern(std::unique_ptr<PwmController>* pwm_controllers,
               uint32_t sample_rate,
               bool is_paused = true);
  virtual ~TreatPattern();
  static std::unique_ptr<TreatPattern> Create(TreatPatternsTypeDef pattern,
                                              bool is_paused = true);
  void Start();
  void Stop();
  void Run();
  
  void SetModDepth(enDepthModCoeffs depth);
  void SetModFreq(enFreqMods freq);
  void SetModRatio(enStagesRatio ratio);
  
  void SetState(bool state);
  bool GetState();
  
  void SetTimer(uint8_t minutes);
  
  uint32_t GetTimerExpiryMS();
 
  template<typename T>
  void SetExpireTimerHandler(T* owner, void (T::*func_ptr)());

protected:
  typedef enum {
    FLEX = 0,
    FIXED,
    PAUSE,
    STAGES_NUM,
    NONE = -1,
  } TreatPatternStagesTypeDef;
  
  struct StageParams {
    TreatPatternStagesTypeDef stage = NONE;
    float time = 1.;
    size_t buf_size = 2 * POINTS_PER_PERIOD_NUM;
  };

  void SampleTimerHandler();
  void BufferReadyHandler(PwmController* pdma);
  void UpdateStageBufferSize(int stage, float mod_freq);
  void TryUpdateDMABuffer(uint32_t stage);
  void SwitchStage();
  void Update(tdUartData msg) override;
  virtual void SetupPattern();
  void OnTimerEnd();
  
  static std::unique_ptr<PwmController>* pwm_controllers_;
  static uint32_t sample_rate_;
  
  tim::HW_STM32_BaseTimer sample_tim_;
  
  int stage_ = 0;
  int stage_num_ = 0;
  bool is_state_on_ = false;
  
  enFreqMods freq_mod_ = FM_10;
  enDepthModCoeffs depth_mod_ = DM_0;
  enStagesRatio stages_ratio_ = APR_10_15;
   
  std::array<StageParams, STAGES_NUM>               stage_params_{};
  std::array<TreatPatternStagesTypeDef, STAGES_NUM> stages_order_{FLEX, NONE, NONE};
  std::vector<StageParams> stages_init_;

  tim::SW_FreeRTOS_Timer treat_timer_;
  std::unique_ptr<_util::GenericCallback<>> expire_callback_;
  
  void SetupStages() {
    int cnt = 0;
    stages_order_ = { FLEX, NONE, NONE };
    for (auto& l : stages_init_) {
      stage_params_[l.stage].stage = l.stage;
      stage_params_[l.stage].time = (STAGES_RATIO[stages_ratio_][cnt]
                                     * (FP_TYPE)l.time);
      stages_order_[cnt] = l.stage;
      ++cnt;
    }
    
    stage_num_ = 0;
    for (auto& s : stages_order_) {
      if (s == NONE) return;
      ++stage_num_;
    }
  }
};

template<typename T>
void TreatPattern::SetExpireTimerHandler(T* owner, void (T::*func_ptr)()) {
  expire_callback_ = std::make_unique<_util::Callback<T>>(owner, func_ptr);
}

// Pattern 1 ================================================================

class PatternContinuous : public TreatPattern {
public:
  PatternContinuous(std::unique_ptr<PwmController>* pwm_controllers, uint32_t sample_rate, bool is_paused)
    : TreatPattern(pwm_controllers, sample_rate, is_paused)
  { 
      SetupPattern(); 
      Start();
  }
private:
  void SetupPattern() override;
};

// Pattern 2 ================================================================

class PatternModulatedPaused : public TreatPattern {
public:
  PatternModulatedPaused(std::unique_ptr<PwmController>* pwm_controllers, uint32_t sample_rate, bool is_paused)
    : TreatPattern(pwm_controllers, sample_rate, is_paused)
  { 
      SetupPattern(); 
      Start();
  }
private:
  void SetupPattern() override;
};

// Pattern 3 ================================================================

class PatternModulatedNonModulated : public TreatPattern {
public:
  PatternModulatedNonModulated(std::unique_ptr<PwmController>* pwm_controllers, uint32_t sample_rate, bool is_paused)
    : TreatPattern(pwm_controllers, sample_rate, is_paused)
  { 
      SetupPattern(); 
      Start();
  }
private:
  void SetupPattern() override;
};

// Pattern 4 ================================================================

class PatternModulatedFixed150 : public TreatPattern {
public:
  PatternModulatedFixed150(std::unique_ptr<PwmController>* pwm_controllers, uint32_t sample_rate, bool is_paused)
    : TreatPattern(pwm_controllers, sample_rate, is_paused)
  { 
      SetupPattern(); 
      Start();
  }
private:
  void SetupPattern() override;
};

// Pattern 5 ================================================================

class PatternModulatedFixed150Paused : public TreatPattern {
public:
  PatternModulatedFixed150Paused(std::unique_ptr<PwmController>* pwm_controllers, uint32_t sample_rate, bool is_paused)
    : TreatPattern(pwm_controllers, sample_rate, is_paused) 
  { 
      SetupPattern(); 
      Start();
  }
private:
  void SetupPattern() override;
};

} // namespace miostim

#endif // #ifndef _TREATMENT_PATTERNS_H_