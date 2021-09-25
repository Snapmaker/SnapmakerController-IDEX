#include "stdint.h"

class Calibration
{
  public:
    bool calibrate_z_offset();

  private:
    void preapare(uint8_t extruder_index);
    float z_probe(float z_distance, uint16_t feedrate);
    float probe(uint8_t axis, float distance, uint16_t feedrate);
    void end();
};

extern class Calibration calibration;
