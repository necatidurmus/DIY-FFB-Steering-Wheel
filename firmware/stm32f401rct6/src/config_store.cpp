#include "config_store.h"

#include <EEPROM.h>
#include <string.h>
#include <utility/stm32_eeprom.h>

namespace {
constexpr uint32_t kConfigAddress = 0U;
}

bool ConfigStore::load(wheel::WheelSettings& settings) const {
  wheel::SettingsRecord record{};
  EEPROM.get(kConfigAddress, record);
  if (!wheel::settingsRecordIsValid(record)) {
    return false;
  }
  settings = record.settings;
  return true;
}

bool ConfigStore::save(const wheel::WheelSettings& settings) const {
  if (!wheel::settingsAreValid(settings)) {
    return false;
  }

  const wheel::SettingsRecord record = wheel::makeSettingsRecord(settings);
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&record);

  // STM32 EEPROM emulasyonu normal EEPROM.put() ile byte basina bir sektor
  // silebilir. Buffered API tek bir RAM guncellemesi ve tek Flash erase/program
  // islemi yapar. Bu fonksiyon yalniz motor dururken cagrilir.
  eeprom_buffer_fill();
  for (size_t index = 0; index < sizeof(record); ++index) {
    eeprom_buffered_write_byte(kConfigAddress + index, bytes[index]);
  }
  eeprom_buffer_flush();

  wheel::SettingsRecord verification{};
  EEPROM.get(kConfigAddress, verification);
  return wheel::settingsRecordIsValid(verification) &&
         memcmp(&record, &verification, sizeof(record)) == 0;
}
