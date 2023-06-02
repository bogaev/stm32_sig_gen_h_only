#ifndef _ARGUMENT_USER_TYPES_H_
#define _ARGUMENT_USER_TYPES_H_

#include <stddef.h>

enum enAutoreload { 
  AUTORELOAD_OFF = 0,
  AUTORELOAD_ON
};

class TimerPeriod {
public:
  explicit TimerPeriod(size_t period_ms) : period_ms_(period_ms) {}
  operator size_t() {
    return period_ms_;
  }
  TimerPeriod& operator=(size_t rhs) {
    period_ms_ = rhs;
    return *this;
  }
private:
  size_t period_ms_;
};

inline TimerPeriod operator""_ms(unsigned long long period_ms) {
  return TimerPeriod(period_ms);
}

#endif // _ARGUMENT_USER_TYPES_H_