#pragma once
#include <stdint.h>
#include "system.h"
#include "../J1/common_type.h"
#include "../debug/debug.h"
#include "../../../Marlin/src/core/macros.h"
#include <EEPROM.h>

#define FACTORY_DATA_MAGIC                  "SNAP"
#define DEFAULT_BUILD_PLATE_THICKNESS       (5.0)
#define MAX_BUILD_PLATE_THICKNESS           (5.5)
#define MIN_BUILD_PLATE_THICKNESS           (4.699)

#pragma pack(push, 1)

typedef struct {
  float build_plate_thickness;
} fd_info_t;

typedef struct{
  uint8_t     magic[4];
  uint32_t    len;
  fd_info_t   fd;
  uint8_t     dump[FACTORY_DATA_SIZE - 12 - sizeof(fd_info_t)];
  uint32_t    checksum;
} fd_flash_t;
#pragma pack(pop)

class factory_data_srv {
public:
  bool init();
  bool load();
  bool save();
  void reset();
  void erase();
  void log();

  bool setBuildPlateThickness(float bpt);
  float getBuildPlateThickness();

private:
  bool check(fd_flash_t &fd);

private:
  fd_flash_t _data;
};

extern factory_data_srv fd_srv;

