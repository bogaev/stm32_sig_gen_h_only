#ifndef _TREATMENT_PATTERNS_EXTRA_H_
#define _TREATMENT_PATTERNS_EXTRA_H_

#include "signal_generator\treatment_patterns.hpp"

namespace miostim {
	
#define DECLARE_TREAT_PATTERN_CLASS(CLASSNAME, CARRIER_SIGNAL_TYPE, CARRIER_FREQ, AMP_MOD_SIGNAL_TYPE, AMP_MOD_FREQ, AMP_DEPTH, FIXED_FREQ_MOD) \
class CLASSNAME : public TreatPattern { \
public: \
  CLASSNAME(std::unique_ptr<PwmController>* pwm_controllers, uint32_t sample_rate, bool is_paused) \
    : TreatPattern(pwm_controllers, sample_rate, is_paused) \
  { \
      SetupPattern(); \
      Start(); \
  } \
private: \
  void SetupPattern() override; \
}; \
\
inline void CLASSNAME::SetupPattern() { \
  pwm_controllers_[FIXED]->SetSignal(SIG_GEN_CARRIER, \
                                 SIG_GEN_PARAM_SIGNAL_TYPE, \
                                 CARRIER_SIGNAL_TYPE); \
\
  pwm_controllers_[FIXED]->SetSignal(SIG_GEN_CARRIER, \
                                 SIG_GEN_PARAM_FREQ, \
                                 CARRIER_FREQ); \
\
  pwm_controllers_[FIXED]->SetSignal(SIG_GEN_AMP_MOD, \
                                 SIG_GEN_PARAM_SIGNAL_TYPE, \
                                 AMP_MOD_SIGNAL_TYPE); \
\
  pwm_controllers_[FIXED]->SetSignal(SIG_GEN_AMP_MOD, \
                                 SIG_GEN_PARAM_FREQ, \
                                 AMP_MOD_FREQ); \
\
  pwm_controllers_[FIXED]->SetSignal(SIG_GEN_AMP_MOD, \
                                 SIG_GEN_PARAM_AMP_DEPTH, \
                                 AMP_DEPTH); \
\
  UpdateStageBufferSize(FIXED, FREQ_MOD[FIXED_FREQ_MOD]); \
  stages_init_ = {{FLEX, MIN_ACT_TIME_MS}, {FIXED, MIN_ACT_TIME_MS}, {PAUSE, MIN_ACT_TIME_MS}}; \
  SetupStages(); \
}

DECLARE_TREAT_PATTERN_CLASS(NewPattern,
														SIG_GEN_TYPE_SINUS,
														mod_values::CARRIER_FREQ,
														SIG_GEN_TYPE_SINUS,
														mod_values::FIXED_MOD_FREQ, 50, freq_mod_)
}

#endif // #ifndef _TREATMENT_PATTERNS_EXTRA_H_