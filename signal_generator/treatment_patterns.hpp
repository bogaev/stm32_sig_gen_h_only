#ifndef _TREATMENT_PATTERNS_H_
#define _TREATMENT_PATTERNS_H_

#include "common_\common.h"
#include "common_\sig_gen_config.h"
#include "signal_generator\pwm_controller.hpp"
#include "utility\observer.hpp"
#include "utility\timer_facility.hpp"
#include "utility\shared_objects.hpp"

#define MAX_PATTERN_STAGES 3

namespace miostim {

using namespace miostim::mod_values;

//class ITreatPattern {
// public:
//  ITreatPattern();
// 
// private:
//  tim::SW_FreeRTOS_Timer stage_tim_;
//}

//inline ITreatPattern::ITreatPattern()
//	: 
//{}

} // namespace miostim

#endif // #ifndef _TREATMENT_PATTERNS_H_