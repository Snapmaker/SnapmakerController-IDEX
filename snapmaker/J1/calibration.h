#include "stdint.h"


#define ADJUST_NOZZLE_LEAD 0.5 // Travel distance of nozzle per revolution, mm
#define ADJUST_NOZZLE_LAP_STEP 10  // A scale that can be turned per turn
#define ADJUST_NOZZLE_MM_PER_STEP (ADJUST_NOZZLE_LEAD / ADJUST_NOZZLE_LAP_STEP)
class Calibration
{
  public:
    bool calibrate_z_offset();
    bool calibrate_nozzle_height();
  private:
    bool probe_nozzle(uint8_t extruder);
    void get_z_probe_height(float *height);
    void preapare(uint8_t extruder_index);
    float z_probe(float z_distance, uint16_t feedrate);
    float probe(uint8_t axis, float distance, uint16_t feedrate);
    void end();
};

extern class Calibration calibration;
