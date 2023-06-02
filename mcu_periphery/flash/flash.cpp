#include "flash.hpp"
#include "eeprom.h"

std::vector<uint16_t> VirtAddVarTab;

namespace flash {

Controller::Controller(std::vector<SerializedClass*> serialized_classes)
  : serialized_classes_(serialized_classes) {
    
//  InitSerializedClasses();
  HAL_FLASH_Unlock();
  if(EE_Init() != EE_OK) {
    Error_Handler();
  }
}

void Controller::Init() {
  uint16_t vaddr = 1;
  std::vector<uint16_t> ret;
  
  for (SerializedClass* cls : serialized_classes_) {
    cls->RegisterController(this);
    auto& data = cls->GetSerializedData();
    for (auto& val : data) {
      val.vaddr = vaddr++;
      VirtAddVarTab.push_back(val.vaddr);
    }
  }
}

void Controller::AddClass(SerializedClass* cls) {
  serialized_classes_.emplace_back(cls);
}

HAL_StatusTypeDef Controller::StoreToFlash(SerializedClass::StoredData& data) { 
  if((EE_WriteVariable(data.vaddr, data.value)) != HAL_OK) {
    return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef Controller::LoadFromFlash(std::vector<SerializedClass::StoredData>& data) {
  for (auto& val : data) {
    if ((EE_ReadVariable(val.vaddr, &val.value)) != HAL_OK) {
      return HAL_ERROR;
    }
  }
  return HAL_OK;
}

} // namespace flash
