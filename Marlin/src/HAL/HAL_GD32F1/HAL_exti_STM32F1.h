#pragma once

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#define PA 0
#define PB 1
#define PC 2
#define PD 3
#define PE 4
#define PF 5
#define PG 6

typedef enum {
    EXTI_Falling,
    EXTI_Rising,
    EXTI_Rising_and_falling
}EXTI_MODE_E;
/**
* ExtiInit:Exti Interrup INIT_AUTO_FAN_PIN
* para PortIndex:GPIOA-GPIOI
* para PinIndex:0-15
* para RisingFallingEdge:0-3,0:Falling, 1:Rising, 2:Rising and falling
*/
void ExtiInit(uint8_t PortIndex, uint8_t PinIndex, uint8_t RisingFallingEdge) ;
void ExtiInit(uint8_t pin, EXTI_MODE_E mode) ;
void EnableExtiInterrupt(uint8_t pin);
void DisableExtiInterrupt(uint8_t pin);
void ExtiClearITPendingBit(uint8_t pin);