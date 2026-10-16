#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include <WheelLogic.h>

class ConfigStore {
 public:
  bool load(wheel::WheelSettings& settings) const;
  bool save(const wheel::WheelSettings& settings) const;
};

#endif  // CONFIG_STORE_H
