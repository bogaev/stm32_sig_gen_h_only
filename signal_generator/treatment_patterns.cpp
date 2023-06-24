#include "treatment_patterns.h"
#include "utility\shared_objects.h"

namespace miostim {
 
std::unique_ptr<PwmController>* TreatPattern::pwm_controllers_{};
uint32_t TreatPattern::sample_rate_{};
  
TreatPattern::TreatPattern(std::unique_ptr<PwmController>* pwm_controllers,
                           uint32_t sample_rate,
                           bool is_state_on)
  : sample_tim_(&htim7, TimerPeriod(MIN_ACT_TIME_MS))
  , is_state_on_(is_state_on)
  , treat_timer_(this, 1_ms, AUTORELOAD_OFF)
{
  pwm_controllers_ = pwm_controllers;
  sample_rate_ = sample_rate;
//  SharedObjects::mod_period_end.RegisterObserver(this);
  SharedObjects::uart_received.RegisterObserver(this);
  sample_tim_.SetExpireTimerHandler(this, &TreatPattern::SampleTimerHandler);
  treat_timer_.SetExpireTimerHandler(this, &TreatPattern::OnTimerEnd);
  for(uint32_t i = 0; i < MAX_PATTERN_STAGES; ++i) {
    auto pwm = dynamic_cast<DMA_PwmController*>(pwm_controllers_[i].get());
    if (pwm) {
      pwm->SetBufferReadyHandler(this, &TreatPattern::BufferReadyHandler);
    }
  }
  SetupPattern();
}

TreatPattern::~TreatPattern() {
  Stop();
//  SharedObjects::mod_period_end.RemoveObserver(this);
  SharedObjects::uart_received.RemoveObserver(this);
}

void TreatPattern::SetTimer(uint8_t minutes) {
  if (minutes == 0) {
    treat_timer_.ChangePeriod( TimerPeriod{1U} );
  } else {
    treat_timer_.ChangePeriod( TimerPeriod{minutes * 1000U * 1U} );
  }
  treat_timer_.Stop();
}

void TreatPattern::OnTimerEnd() {
  Stop();
  if (expire_callback_->isValid()) {
    expire_callback_->execute();
  }
}

uint32_t TreatPattern::GetTimerExpiryMS() {
  return treat_timer_.GetExpiryTimeMS();
}

void TreatPattern::SampleTimerHandler() {
  if (stage_num_ > 1) {
    SwitchStage();
  }
}

void TreatPattern::BufferReadyHandler(PwmController* pdma) {
  if (is_state_on_) {
    if (pwm_controllers_[stage_].get() == pdma) {
      pdma->Run();
    }
    sample_tim_.Start();
  }
}

void TreatPattern::UpdateStageBufferSize(int stage, float mod_freq) {
  float new_buf = ( (mod_values::CARRIER_FREQ - mod_freq) / mod_freq ) 
                      * POINTS_PER_PERIOD_NUM;
  new_buf = (new_buf == 0 ? 2. * POINTS_PER_PERIOD_NUM : new_buf);
  stage_params_[stage].buf_size = (size_t) (2. * new_buf);
}

void TreatPattern::TryUpdateDMABuffer(uint32_t stage) {
  auto pwm = dynamic_cast<DMA_PwmController*>(pwm_controllers_[stage].get());
  if (pwm) {
    Stop();
    pwm->UpdateBufferSize(stage_params_[stage].buf_size);
    pwm->StartBufferUpdate();
  }
}

void TreatPattern::Start() {
  for(uint32_t i = 0; i < MAX_PATTERN_STAGES; ++i) {
    TryUpdateDMABuffer(i);
  }
}

void TreatPattern::Stop() {
  sample_tim_.Stop();
  for(uint32_t i = 0; i < MAX_PATTERN_STAGES; ++i) {
    pwm_controllers_[i]->Stop();
  }
  treat_timer_.Stop();
}

void TreatPattern::Run() {
//  if (is_state_on_) {
    for(uint32_t i = 0; i < MAX_PATTERN_STAGES; ++i) {
      pwm_controllers_[i]->Run();
    }
    sample_tim_.Start();
//    if (treat_timer_.GetTimeMS() > 1) {
//      treat_timer_.Start();
//    }
//  }
}

void TreatPattern::SetState(bool state) {
  is_state_on_ = state;
  
  if (is_state_on_) {
    Run();
    if (treat_timer_.GetTimeMS() > 1) {
      treat_timer_.Start();
    }
  } else {
    Stop();
  }
}

bool TreatPattern::GetState() {
  return is_state_on_;
}

std::unique_ptr<TreatPattern> TreatPattern::Create(TreatPatternsTypeDef pattern,
                                                   bool is_state_on) {
  if (pattern == TREAT_PATTERN_MODULATED_CONTINUOUS) {
    return std::make_unique<PatternContinuous>(pwm_controllers_, sample_rate_, is_state_on);
  } else if (pattern == TREAT_PATTERN_MODULATED_PAUSED) {
    return std::make_unique<PatternModulatedPaused>(pwm_controllers_, sample_rate_, is_state_on);
  } else if (pattern == TREAT_PATTERN_MODULATED_NON_MODULATED) {
    return std::make_unique<PatternModulatedNonModulated>(pwm_controllers_, sample_rate_, is_state_on);
  } else if (pattern == TREAT_PATTERN_MODULATED_FIXED_150) {
    return std::make_unique<PatternModulatedFixed150>(pwm_controllers_, sample_rate_, is_state_on);
  } else if (pattern == TREAT_PATTERN_MODULATED_FIXED_150_PAUSED) {
    return std::make_unique<PatternModulatedFixed150Paused>(pwm_controllers_, sample_rate_, is_state_on);
  }
  return std::make_unique<PatternContinuous>(pwm_controllers_, sample_rate_, is_state_on);
}

void TreatPattern::Update(tdUartData msg) {
  pwm_controllers_[FLEX]->SetSignal(msg.signal, msg.param, msg.value);
}

void TreatPattern::SetModFreq(enFreqMods freq) {
  if (freq_mod_ != freq) {
    freq_mod_ = freq;

    pwm_controllers_[FLEX]->SetSignal(SIG_GEN_AMP_MOD,
                                      SIG_GEN_PARAM_FREQ,
                                      FREQ_MOD[freq_mod_]);
    
    UpdateStageBufferSize(FLEX, FREQ_MOD[freq_mod_]);
    TryUpdateDMABuffer(FLEX);
  }
}

void TreatPattern::SetModDepth(enDepthModCoeffs depth) {
  if (depth_mod_ != depth) {
    depth_mod_ = depth;

    pwm_controllers_[FLEX]->SetSignal(SIG_GEN_AMP_MOD,
                                      SIG_GEN_PARAM_AMP_DEPTH,
                                      DEPTH_MOD_PERCENT[depth_mod_]);
    TryUpdateDMABuffer(FLEX);
  }
}

void TreatPattern::SetModRatio(enStagesRatio ratio) {
  if (stages_ratio_ != ratio) {
    stages_ratio_ = ratio;
    SetupStages();
  }
}

void TreatPattern::SwitchStage() {
  static int cnt = 0;
  stage_ = stages_order_[(++cnt) % stage_num_];
  Stop();
  sample_tim_.ChangePeriod(TimerPeriod((size_t)stage_params_[stage_].time));
  if (stage_ != PAUSE) {
    pwm_controllers_[stage_]->Run();
  }
  sample_tim_.Start();
}

void TreatPattern::SetupPattern() {
  
  pwm_controllers_[FLEX]->SetSignal(SIG_GEN_CARRIER,
                                    SIG_GEN_PARAM_SIGNAL_TYPE,
                                    SIG_GEN_TYPE_SINUS);

  pwm_controllers_[FLEX]->SetSignal(SIG_GEN_CARRIER,
                                    SIG_GEN_PARAM_FREQ,
                                    mod_values::CARRIER_FREQ);

  pwm_controllers_[FLEX]->SetSignal(SIG_GEN_AMP_MOD,
                                    SIG_GEN_PARAM_SIGNAL_TYPE,
                                    SIG_GEN_TYPE_SINUS);
  
  pwm_controllers_[FLEX]->SetSignal(SIG_GEN_AMP_MOD,
                                    SIG_GEN_PARAM_FREQ,
                                    FREQ_MOD[freq_mod_]);
  
  pwm_controllers_[FLEX]->SetSignal(SIG_GEN_AMP_MOD,
                                    SIG_GEN_PARAM_AMP_DEPTH,
                                    DEPTH_MOD_PERCENT[depth_mod_]);
  
  UpdateStageBufferSize(FLEX, FREQ_MOD[freq_mod_]);
  stages_init_ = {{FLEX, MIN_ACT_TIME_MS}};
  SetupStages();
}

// Pattern 1 ================================================================

void PatternContinuous::SetupPattern() {
  stages_init_ = { {FLEX, MIN_ACT_TIME_MS} };
  SetupStages();
}

// Pattern 2 ================================================================

void PatternModulatedPaused::SetupPattern() {
  stages_init_ = { {FLEX, MIN_ACT_TIME_MS}, {PAUSE, MIN_ACT_TIME_MS} };
  SetupStages();
}

// Pattern 3 ================================================================

void PatternModulatedNonModulated::SetupPattern() {

  pwm_controllers_[FIXED]->SetSignal(SIG_GEN_CARRIER,
                                 SIG_GEN_PARAM_SIGNAL_TYPE,
                                 SIG_GEN_TYPE_SINUS);
  
  pwm_controllers_[FIXED]->SetSignal(SIG_GEN_CARRIER,
                                 SIG_GEN_PARAM_FREQ,
                                 mod_values::CARRIER_FREQ);
  
  UpdateStageBufferSize(FIXED, FREQ_MOD[freq_mod_]);
  stages_init_ = { {FLEX, MIN_ACT_TIME_MS}, {FIXED, MIN_ACT_TIME_MS} };
  SetupStages();
}

// Pattern 4 ================================================================

void PatternModulatedFixed150::SetupPattern() {
  
  pwm_controllers_[FIXED]->SetSignal(SIG_GEN_CARRIER,
                                 SIG_GEN_PARAM_SIGNAL_TYPE,
                                 SIG_GEN_TYPE_SINUS);
  
  pwm_controllers_[FIXED]->SetSignal(SIG_GEN_CARRIER,
                                 SIG_GEN_PARAM_FREQ,
                                 mod_values::CARRIER_FREQ);
  
  pwm_controllers_[FIXED]->SetSignal(SIG_GEN_AMP_MOD,
                                 SIG_GEN_PARAM_SIGNAL_TYPE,
                                 SIG_GEN_TYPE_SINUS);
  
  pwm_controllers_[FIXED]->SetSignal(SIG_GEN_AMP_MOD,
                                 SIG_GEN_PARAM_FREQ,
                                 mod_values::FIXED_MOD_FREQ);
  
  pwm_controllers_[FIXED]->SetSignal(SIG_GEN_AMP_MOD,
                                 SIG_GEN_PARAM_AMP_DEPTH,
                                 50);
  
  UpdateStageBufferSize(FIXED, FREQ_MOD[freq_mod_]);
  stages_init_ = {{FLEX, MIN_ACT_TIME_MS}, {FIXED, MIN_ACT_TIME_MS}};
  SetupStages();
}

// Pattern 5 ================================================================

void PatternModulatedFixed150Paused::SetupPattern() {
  
  pwm_controllers_[FIXED]->SetSignal(SIG_GEN_CARRIER,
                                 SIG_GEN_PARAM_SIGNAL_TYPE,
                                 SIG_GEN_TYPE_SINUS);
  
  pwm_controllers_[FIXED]->SetSignal(SIG_GEN_CARRIER,
                                 SIG_GEN_PARAM_FREQ,
                                 mod_values::CARRIER_FREQ);
  
  pwm_controllers_[FIXED]->SetSignal(SIG_GEN_AMP_MOD,
                                 SIG_GEN_PARAM_SIGNAL_TYPE,
                                 SIG_GEN_TYPE_SINUS);
  
  pwm_controllers_[FIXED]->SetSignal(SIG_GEN_AMP_MOD,
                                 SIG_GEN_PARAM_FREQ,
                                 mod_values::FIXED_MOD_FREQ);
  
  pwm_controllers_[FIXED]->SetSignal(SIG_GEN_AMP_MOD,
                                 SIG_GEN_PARAM_AMP_DEPTH,
                                 50);
  
  UpdateStageBufferSize(FIXED, FREQ_MOD[freq_mod_]);
  stages_init_ = {{FLEX, MIN_ACT_TIME_MS}, {FIXED, MIN_ACT_TIME_MS}, {PAUSE, MIN_ACT_TIME_MS}};
  SetupStages();
}

} // namespace miostim
