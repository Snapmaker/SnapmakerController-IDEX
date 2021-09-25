
/**
 * Marlin 3D Printer Firmware
 */
#include "src/inc/MarlinConfigPre.h"
#include "src/core/millis_t.h"
#include <src/pins/pins.h>
#include "tmc_driver.h"
#include "tmc_regs.h"
#include "HAL.h"
// #include "src/HAL/STM32_F4_F7/ExtiInterrupt.h"
// #include "src/jf_modules/JFMachineStatus.h"
// #include "src/jf_modules/switch_detect.h"

extern TMCDriver tmc_driver;

#define SELA_PIN  TMC_SEL0_PIN
#define SELB_PIN  TMC_SEL1_PIN
#define SELC_PIN  TMC_SEL2_PIN

uint8_t TMCDriver::sel_table[8][3] = { 
  //SELA, SELB， SELC
  { LOW,    HIGH,    LOW},    //X1
  { LOW,    LOW,     HIGH},   //X2
  { HIGH,   LOW,     LOW},    //Y1
  { HIGH,   HIGH,    LOW},    //Z
  { LOW,    HIGH,    HIGH},   //E0
  { HIGH,   HIGH,    HIGH},   //E1
  { LOW,    LOW,     LOW},    //Unselect
  { LOW,    LOW,     LOW}     //Unselect
};

uint8_t TMCDriver::slave_address[] = {
  0x03,
  0x03,
  0x03,
  0x03,
  0x03,
  0x03
};

uint8_t TMCDriver::stall_guard_level_table[][4] = {
  {1, 1, 1, 1},
  {5, 5, 5, 1},
  {10, 10, 14, 1},
  {15, 15, 17, 1},
  {17, 17, 24, 1}
};

uint8_t TMCDriver::select_index = 0xff;
uint8_t TMCDriver::recv_buff[16];
uint8_t TMCDriver::send_buff[8];
bool TMCDriver::stall_guard_dectected = false;
SG_Mode TMCDriver::stall_guard_mode = SG_MODE_NORMAL;
uint8_t TMCDriver::stall_trigged_mode = SG_MODE_NONE;
uint8_t TMCDriver::print_stall_guard_level = 1;
uint32_t TMCDriver::stepper_isr_tick_check_threshold = 450;
uint32_t TMCDriver::stepper_isr_tick = 500;

extern HardwareSerial Serial2;

tmc_configure_t TMCDriver::local_configures[] = {
  {R_GCONF,       0x0C9},
  {R_IHOLD_IRUN,  (2<<16) | (24U<<8) | (24U)}, //2 clock delay, full current , 70% standstill
  //{R_TPOWER_DOWN, 20},
  {R_PWMCONF, },
  {R_CHOPCONF, },
  {R_SGTHRS, },
  {R_GCONF,       0x089},
  {(uint8_t)0,  (uint32_t)0}
};

/**
  * @brief  Initialize  
  * @retval None
  */
void TMCDriver::init() {
  OUT_WRITE(SELA_PIN, LOW);
  OUT_WRITE(SELB_PIN, LOW);
  OUT_WRITE(SELC_PIN, LOW);
  Serial2.begin(57600);
  Serial2.setTimeout(350);
//   ExtiInit(TMC_STALL_GUARD_PIN, EXTI_Rising);
//   disable_stall_guard_interrupt();
}

/**
  * @brief  Configure for axis test
  * @param  index: Index of the axis, 0 for X0, 1 for X1, 2 for Y, 3 for Z
  * @param  sg_value: Stall guard value
  * @retval None
  */
void TMCDriver::configure_for_aixs_test(uint8_t index, uint8_t sg_value) {
  set_reg_value(index, R_TCOOLTHRS, 0x200);
  stall_guard_init(index, sg_value);
  set_stall_guard_mode((SG_Mode)(index + SG_MODE_X0_HOME));
}

/**
  * @brief  Configure for printing
  * @retval None
  */
void TMCDriver::configure_for_print() {
  stall_guard_init(0, stall_guard_level_table[print_stall_guard_level][0]);
  stall_guard_init(1, stall_guard_level_table[print_stall_guard_level][1]);
  stall_guard_init(2, stall_guard_level_table[print_stall_guard_level][2]);
  stall_guard_init(3, stall_guard_level_table[print_stall_guard_level][3]);
  set_stall_guard_mode(SG_MODE_NORMAL);
  set_stepper_tick_threshold(370);
  enable_stall_guard();
  // JFMachineStatus::ClearMovementFlag();
}

/**
  * @brief  Configure for XY Calibration
  * @retval None
  */
void TMCDriver::configure_for_xy_calibration(uint8_t sg_x, uint8_t sg_y) {
  stall_guard_init(0, sg_x);
  stall_guard_init(1, sg_x);
  stall_guard_init(2, sg_y);
  stall_guard_init(3, 5);
  // cool_step_init(0, false, 0, 0);
  set_stall_guard_mode(SG_MODE_CALIBRATION);
  set_stepper_tick_threshold(500);
  enable_stall_guard();
  // JFMachineStatus::ClearCalibrateFault();
  // JFMachineStatus::ClearMovementFlag();
  select(7);
}

/**
  * @brief  Configure for Platform Calibration
  * @retval None
  */
void TMCDriver::configure_for_platform_calibration(uint8_t sg_z) {
  stall_guard_init(0, 5);
  stall_guard_init(1, 5);
  stall_guard_init(2, 5);
  stall_guard_init(3, sg_z);
  // cool_step_init(3, false, 0, 0);
  set_stall_guard_mode(SG_MODE_CALIBRATION);
  enable_stall_guard();
  set_stepper_tick_threshold(400);
  // JFMachineStatus::ClearCalibrateFault();
  // JFMachineStatus::ClearMovementFlag();
}

/**
  * @brief  Configure for idle
  * @retval None
  */
void TMCDriver::configure_for_idle() {
  stall_guard_deinit(0);
  stall_guard_deinit(1);
  stall_guard_deinit(2);
  stall_guard_deinit(3);
  set_stall_guard_mode(SG_MODE_NONE);
  enable_stall_guard();
  // JFMachineStatus::ClearMovementFlag();
}

/**
  * @brief    
  * @param   
  * @retval None
  */
void TMCDriver::stall_guard_init(uint8_t index, uint8_t sg_value) {
  uint32_t tmp_value;
  tmp_value = sg_value;
  set_reg_value(index, R_SGTHRS, tmp_value);
}

/**
  * @brief  Se t coolstep
  * @param  index: Index of the motor
  * @param  enable: Enable or disable the function. If set false, will ignore LowLimit and HighLimit
  * @param  low_limit: The Low threshold
  * @param  high_limit: The Higt threshold
  * @retval None
  */
void TMCDriver::cool_step_init(uint8_t index, bool enable, uint8_t low_limit, uint8_t high_limit) {
  uint32_t tmp_value;
  
  TMCDriver::select(index);
  if(enable == false) {
    tmp_value = 0;
    TMCDriver::comm_test(0, R_COOLCONF, &tmp_value);
  }
  else {
    tmp_value = ((high_limit << 8) & 0xff00) | (low_limit & 0xff);
    TMCDriver::comm_test(0, R_COOLCONF, &tmp_value);
  }
}

/**
  * @brief    
  * @param   
  * @retval None
  */
void TMCDriver::stall_guard_deinit(uint8_t index) {
  set_reg_value(index, R_SGTHRS, 5);
}

/**
  * @brief  Set register value
  * @param  index: Motor index
  * @param  reg: Register address
  * @param  value: Value to set
  * @retval None
  */
void TMCDriver::set_reg_value(uint8_t index, uint8_t reg, uint32_t value) {
  millis_t tmp_tick;
  select(index);
  write_reg(reg, value);
  // Write reg must delay
  tmp_tick = millis() + 450;
  while(tmp_tick > millis());
}

/**
  * @brief  Initialize all axis
  * @retval None
  */
void TMCDriver::configure_axis() {
  uint32_t reg_value;

  reg_value = (4 << 16) | (16 << 8) | (6);

  // X Axis
  for(int i=0;i<2;i++) {
    set_reg_value(i, R_GCONF, 0x141);
    set_reg_value(i, R_TCOOLTHRS, 0x300);
    set_reg_value(i, R_COOLCONF, 0xa122);
    set_reg_value(i, R_TPWMTHRS, 8);
    set_reg_value(i, R_IHOLD_IRUN, reg_value);
    // set_reg_value(i, R_GCONF, 0x101);
    set_reg_value(i, R_SGTHRS, 65);
  }

  // Y Axis
  reg_value = (7 << 16) | (25 << 8) | (16);
  set_reg_value(2, R_GCONF, 0x141);
  set_reg_value(2, R_TCOOLTHRS, 0x300);
  set_reg_value(2, R_TPWMTHRS, 8);
  set_reg_value(2, R_IHOLD_IRUN, reg_value);
  // set_reg_value(2, R_GCONF, 0x101);
  set_reg_value(2, R_SGTHRS, 5);

  // Z Axis
  reg_value = (7 << 16) | (28 << 8) | (16);
  set_reg_value(3, R_GCONF, 0x141);
  set_reg_value(3, R_TCOOLTHRS, 0x300);
  set_reg_value(3, R_TPWMTHRS, 16);
  set_reg_value(3, R_IHOLD_IRUN, reg_value);
  // set_reg_value(3, R_GCONF, 0x101);
  set_reg_value(3, R_SGTHRS, 5);

  reg_value = (7 << 16) | (15 << 8) | (16);
  // Extrder left
  set_reg_value(4, R_GCONF, 0x141);
  set_reg_value(4, R_IHOLD_IRUN, reg_value);

  // Extrder right
  set_reg_value(5, R_GCONF, 0x141);
  set_reg_value(5, R_IHOLD_IRUN, reg_value);

  select(7);
}

/**
  * @brief  Write config to the driver
  * @param  index: Driver index
  * @param  configure: The config set to write
  * @retval None
  */
void TMCDriver::configure(uint8_t index, tmc_configure_t *configure) {
  select(index);
  write_reg(configure->RegAddress, configure->Value);
}

/**
  * @brief    
  * @param  rw: Read write flag, 0 for write, 1 for read
  * @param  reg_address: Register address
  * @param  value: The pointer to the value
  * @retval None
  */
void TMCDriver::comm_test(uint8_t rw, uint8_t reg_address, uint32_t *value) {
  millis_t tmp_tick;
  switch(rw) {
    case 0:
      write_reg(reg_address, *value);
    break;

    case 1:
      *value = read_reg(reg_address);
    break;
  }
  tmp_tick = millis() + 20;
  while(tmp_tick > millis());
}

/**
  * @brief  Write value to the register
  * @param  reg_address: Register address
  * @param  value: The value to write
  * @retval None
  */
void TMCDriver::write_reg(uint8_t reg_address, uint32_t value) {
  // Serial2.reset_tb();
  send_buff[0] = 0x05;
  send_buff[1] = slave_address[select_index];
  send_buff[2] = reg_address | 0x80;
  send_buff[3] = (value >> 24) & 0xff;
  send_buff[4] = (value >> 16) & 0xff;
  send_buff[5] = (value >> 8) & 0xff;
  send_buff[6] = (value >> 0) & 0xff;
  cacul_crc(send_buff, 8);
  Serial2.write(send_buff, 8);
  // Serial2.reset_rb();
}

/**
  * @brief  Read the value of the register
  * @param  reg_address: Register address
  * @retval The value of the register
  */
uint32_t TMCDriver::read_reg(uint8_t reg_address) {
  uint32_t reg_value = 0xff;

  // Serial2.reset_tb();
  // Serial2.reset_rb();

  send_buff[0] = 0x05;
  send_buff[1] = slave_address[select_index];
  send_buff[2] = reg_address & 0x7f;
  cacul_crc(send_buff, 4);
  Serial2.write(send_buff, 4);

  // First 4 bytes are sent by host
  if(Serial2.readBytes(recv_buff, 12) == 12) {
      reg_value = (recv_buff[7] << 24) | (recv_buff[8] << 16) | (recv_buff[9] << 8) | (recv_buff[10]);
  }
  return reg_value;
}

/**
  * @brief  Select motor
  * @param  index
  * @retval None
  */
void TMCDriver::select(uint8_t index) {
  if(select_index == index)
    return;
  OUT_WRITE(SELA_PIN, LOW);
  OUT_WRITE(SELB_PIN, LOW);
  OUT_WRITE(SELC_PIN, LOW);

  if(index < (sizeof(sel_table) / sizeof(sel_table[0]))) {
    OUT_WRITE(SELA_PIN, sel_table[index][0]);
    OUT_WRITE(SELB_PIN, sel_table[index][1]);
    OUT_WRITE(SELC_PIN, sel_table[index][2]);
    select_index = index;
  }
  // Delay for changing channel
  millis_t tmp_tick;
  tmp_tick = millis() + 100;
  while(tmp_tick > millis());
}

/**
  * @brief  RCR caculate from TMC specification
  * @param  datagram
  * @param  datagramLength
  * @retval None
  */
void TMCDriver::cacul_crc(uint8_t* datagram, uint8_t datagramLength)
{
  int i,j;
  uint8_t* crc = datagram + (datagramLength-1); // CRC located in last byte of message
  uint8_t currentByte;
  *crc = 0;
  for (i=0; i<(datagramLength-1); i++) { // Execute for all bytes of a message
    currentByte = datagram[i]; // Retrieve a byte to be sent from Array
    for (j=0; j<8; j++) {
      if ((*crc >> 7) ^ (currentByte & 0x01)) // update CRC based result of XOR operation
      {
        *crc = (*crc << 1) ^ 0x07;
      }
      else
      {
        *crc = (*crc << 1);
      }
      currentByte = currentByte >> 1;
    } // for CRC bit
  } // for message byte
}

/**
  * @brief  Enable stall guard
  * @retval None
  */
void TMCDriver::stall_guard_check_loop() {
  if(stall_guard_dectected == true) {
    // if(stall_trigged_mode == SG_MODE_X0_HOME)
    //   JFMachineStatus::SetFaultFlag(FAULT_FLAG_X0_CARRIER);
    // else if(stall_trigged_mode == SG_MODE_X1_HOME)
    //   JFMachineStatus::SetFaultFlag(FAULT_FLAG_X1_CARRIER);
    // else if(stall_trigged_mode == SG_MODE_Y_HOME)
    //   JFMachineStatus::SetFaultFlag(FAULT_FLAG_Y_AXIS);
    // else if(stall_trigged_mode == SG_MODE_Z_HOME)
    //   JFMachineStatus::SetFaultFlag(FAULT_FLAG_Z_AXIS);
    // else if(stall_trigged_mode == SG_MODE_NORMAL)
    //   JFMachineStatus::SetFaultFlag(FAULT_FLAG_MOVMENT);
    // else if(stall_trigged_mode == SG_MODE_CALIBRATION)
    //   JFMachineStatus::SetFaultFlag(FAULT_FLAG_CALIBRATE);
    stall_trigged_mode = SG_MODE_NONE;
    stall_guard_dectected = false;
  }
}

/**
  * @brief  Enable stall guard
  * @retval None
  */
void TMCDriver::set_stall_guard_mode(SG_Mode mode) {
  stall_guard_mode = mode;
}

/**
  * @brief  Enable stall guard
  * @retval None
  */
void TMCDriver::enable_stall_guard() {
  enable_stall_guard_interrupt();
  stall_guard_dectected = false;
}

/**
  * @brief  Disable stall guard
  * @retval None
  */
void TMCDriver::disable_stall_guard() {
  disable_stall_guard_interrupt();
  stall_guard_dectected = false;
}

/**
  * @brief  Disable stall guard
  * @retval None
  */
void TMCDriver::trigger_stall_guard() {
  stall_guard_dectected = true;
  stall_trigged_mode = stall_guard_mode;
  stall_guard_check_loop();
}

/**
  * @brief  Set stall guard level
  * @param  level: Level, 0-4
  * @retval None
  */
void TMCDriver::set_sta_guar_level(uint8_t level) {
  print_stall_guard_level = level;
}

/**
  * @brief  Update current stepper tick
  * @param  tick:
  * @retval None
  */
void TMCDriver::update_stepper_tick(uint32_t tick) {
  stepper_isr_tick = tick;
}

/**
  * @brief  Set stepper tick threshold
  * @param  tick: 
  * @retval None
  */
void TMCDriver::set_stepper_tick_threshold(uint32_t tick) {
  stepper_isr_tick_check_threshold = tick;
}

/**
  * @brief  Check if need to check stall
  * @retval True for need check
  */
bool TMCDriver::check_stepper_tick() {
  if(stepper_isr_tick < stepper_isr_tick_check_threshold)
    return true;
  else
    return false;
}

void TMCDriver::enable_stall_guard_interrupt() {
  EnableExtiInterrupt(TMC_STALL_GUARD_PIN);
}

void TMCDriver::disable_stall_guard_interrupt() {
  DisableExtiInterrupt(TMC_STALL_GUARD_PIN);
}


extern "C" {
  void TMCStallGuardHandler() {
    // TMCDriver::trigger_stall_guard();
  }
  void EXTI3_IRQHandler() {
    TMCStallGuardHandler();
    ExtiClearITPendingBit(TMC_STALL_GUARD_PIN);
  }
}
