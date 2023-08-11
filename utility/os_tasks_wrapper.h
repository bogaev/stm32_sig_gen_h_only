#ifndef _OS_TASKS_WRAPPER_H_
#define _OS_TASKS_WRAPPER_H_

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "main.h"
#include "cmsis_os.h"

#include <functional>

class RTOSTaskWrapper {
 public:
  RTOSTaskWrapper(const char* task_name,
                  uint32_t stack_size,
                  osPriority_t priority,
                  std::function<void(void*)> task_code);
  ~RTOSTaskWrapper();

  void Create();
  void Delete();

 private:
  std::function<void(void*)> task_code_;
  osThreadAttr_t attr_{0};
  osThreadId_t handle_;

  static void TaskWrapper(void* argument);
};

inline RTOSTaskWrapper::RTOSTaskWrapper(const char* task_name,
                                 uint32_t stack_size,
                                 osPriority_t priority,
                                 std::function<void(void*)> task_code)
  : task_code_(std::move(task_code))
{
  attr_.name = task_name;
  attr_.stack_size = stack_size;
  attr_.priority = priority;
}

inline RTOSTaskWrapper::~RTOSTaskWrapper() {
  Delete();
}

inline void RTOSTaskWrapper::Create() {
  handle_ = osThreadNew(TaskWrapper, this, &attr_);
}

inline void RTOSTaskWrapper::Delete() {
  osThreadTerminate(handle_);
}

inline void RTOSTaskWrapper::TaskWrapper(void* argument) {
  static_cast<RTOSTaskWrapper*>(argument)->task_code_(nullptr);
}

#endif // #ifndef _OS_TASKS_WRAPPER_H_