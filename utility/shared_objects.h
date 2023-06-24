#ifndef _SHARED_OBJECTS_H_
#define _SHARED_OBJECTS_H_

#include "utility\observer.hpp"
#include "common_\common.h"

struct SharedObjects {
  inline static _pattern::Publisher<tdUartData> uart_received{};
  inline static _pattern::Publisher<tdSignalParams> signal_updater{};
};

#endif // _SHARED_OBJECTS_H_