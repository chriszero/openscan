/*
 * openscan firmware
 * Copyright (C) Christian Völlinger  2019
 *
 * Based on orginal openscan firmware
 * Copyright (C) 2019 Thomas Megel
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "eeprom-config.h"
#include <EEPROM.h>

const StoreStruct default_config = DEFAULT_CONFIG;
StoreStruct user_config = DEFAULT_CONFIG;

void loadConfig() {
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
      EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
      EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2]) {
      for (unsigned int t=0; t<sizeof(user_config); t++)
        *((char*)&user_config + t) = EEPROM.read(CONFIG_START + t);
    } else {
      resetToDefaultConfig();
    }
}

void resetToDefaultConfig() {
  for (unsigned int t=0; t<sizeof(default_config); t++)
    EEPROM.write(CONFIG_START + t, *((char*)&default_config + t));
  loadConfig();
}

void saveConfig() {
  for (unsigned int t=0; t<sizeof(user_config); t++)
    EEPROM.write(CONFIG_START + t, *((char*)&user_config + t));
}