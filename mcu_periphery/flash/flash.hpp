#ifndef _FLASH_H_
#define _FLASH_H_

#include "main.h"
#include "cmsis_os.h"

#include <initializer_list>
#include <vector>

namespace flash {

class Controller;
  
class SerializedClass {
public:
  struct StoredData {
    uint16_t vaddr;
    uint16_t value;
  };
  
  virtual std::vector<StoredData>& GetSerializedData() = 0;
  virtual void Serialize(uint16_t idx, uint16_t data) = 0;
  virtual void Deserialize() = 0;
  void RegisterController(flash::Controller* obj) {
    flash_ = obj;
  }
  
protected:
  std::vector<StoredData> sdata_;
  flash::Controller* flash_ = nullptr;
};

struct InitSettings {
  std::initializer_list<SerializedClass*> serialized_classes;
};

class Controller {
public:
  Controller(std::vector<SerializedClass*> serialized_classes);
  
  void Init();
  void AddClass(SerializedClass* serialized_class);
  HAL_StatusTypeDef StoreToFlash(SerializedClass::StoredData& data);
  HAL_StatusTypeDef LoadFromFlash(std::vector<SerializedClass::StoredData>& data);
    
private:
//  void InitSerializedClasses();
  std::vector<SerializedClass*> serialized_classes_;
};

} // namespace flash

#endif /* _FLASH_H_ */
