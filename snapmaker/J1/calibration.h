#include "stdint.h"


#define ADJUST_NOZZLE_LEAD 0.5 // Travel distance of nozzle per revolution, mm
#define ADJUST_NOZZLE_LAP_STEP 10  // A scale that can be turned per turn
#define ADJUST_NOZZLE_MM_PER_STEP (ADJUST_NOZZLE_LEAD / ADJUST_NOZZLE_LAP_STEP)

#define ADJUST_BED_LEAD 0.5 // Travel distance of bed per revolution, mm
#define ADJUST_BED_LAP_STEP 10  // A scale that can be turned per turn
#define ADJUST_BED_MM_PER_STEP (ADJUST_NOZZLE_LEAD / ADJUST_NOZZLE_LAP_STEP)

#define Z_CAIL_FEEDRATE 4000
#define XY_CAIL_FEEDRATE 400

class Calibration
{
  public:
    bool calibrate_z_offset();
    bool calibrate_nozzle_height();
    bool calibrate_platform();
    bool calibrate_xy();
    bool calibrate_move_xy(uint8_t number);
    void set_build_plate_thickness(float thickness);
    float get_build_plate_thickness();
  private:
    bool probe_nozzle(uint8_t extruder);
    bool probe_xy(uint8_t extruder_index);
    void get_z_probe_height(float *height);
    void preapare(uint8_t extruder_index);
    float x_probe(float x_distance, uint16_t feedrate);
    float y_probe(float y_distance, uint16_t feedrate);
    float z_probe(float z_distance, uint16_t feedrate);
    float probe(uint8_t axis, float distance, uint16_t feedrate);
    void x_home();
    bool bed_leveling_probe(uint8_t extruder_index);
    void move_to_bed_calibration_point(uint8_t index);
    void end();
};

extern class Calibration calibration;
