/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 * Copyright (c) 2011, 2012 LeafLabs, LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file wirish/HardwareSerial.cpp
 * @brief Wirish serial port implementation.
 */

#include "HardwareSerial.h"

#include <libmaple/libmaple.h>
#include <libmaple/gpio.h>
#include <libmaple/timer.h>
#include <libmaple/usart.h>
#include "../../../../debug/debug.h"
HardwareSerial::HardwareSerial(usart_dev *usart_device,
                               uint8 tx_pin,
                               uint8 rx_pin) {
    this->usart_device = usart_device;
    this->tx_pin = tx_pin;
    this->rx_pin = rx_pin;
}

#define BB_BUF_SIZE 256

struct BangBangBuffer {
    uint32_t index;
    uint8_t write_p;
    uint8_t read_p;
    char buffer[2][BB_BUF_SIZE];

    void swap() {
        char temp = write_p;
        write_p   = read_p;
        read_p    = temp;
        index     = 0;
    }

    int write(char c) {
        if (index < BB_BUF_SIZE - 1) {
            buffer[write_p][index++] = c;
            return 1;
        }

        return -1;
    }

    int set(uint32_t i, char c) {
        if (i < BB_BUF_SIZE) {
            buffer[write_p][i] = c;
            return 1;
        }
        else
            return -1;
    }

    char *get_read() {
        return buffer[read_p];
    }
};

static struct BangBangBuffer logger = {0, 0, 1};

/*
 * Set up/tear down
 */

#if STM32_MCU_SERIES == STM32_SERIES_F1
/* F1 MCUs have no GPIO_AFR[HL], so turn off PWM if there's a conflict
 * on this GPIO bit. */
static void disable_timer_if_necessary(timer_dev *dev, uint8 ch) {
    if (dev != NULL) {
        timer_set_mode(dev, ch, TIMER_DISABLED);
    }
}
#elif (STM32_MCU_SERIES == STM32_SERIES_F2) ||    \
      (STM32_MCU_SERIES == STM32_SERIES_F4)
#define disable_timer_if_necessary(dev, ch) ((void)0)
#else
#warning "Unsupported STM32 series; timer conflicts are possible"
#endif

void HardwareSerial::begin(uint32 baud)
{
	begin(baud,SERIAL_8N1);
}
/*
 * Roger Clark.
 * Note. The config parameter is not currently used. This is a work in progress.
 * Code needs to be written to set the config of the hardware serial control register in question.
 *
*/

void HardwareSerial::begin(uint32 baud, uint8_t config)
{
 //   ASSERT(baud <= this->usart_device->max_baud);// Roger Clark. Assert doesn't do anything useful, we may as well save the space in flash and ram etc

    if (baud > this->usart_device->max_baud) {
        return;
    }

    const stm32_pin_info *txi = &PIN_MAP[this->tx_pin];
    const stm32_pin_info *rxi = &PIN_MAP[this->rx_pin];

    disable_timer_if_necessary(txi->timer_device, txi->timer_channel);

    usart_init(this->usart_device);
    usart_config_gpios_async(this->usart_device,
                             rxi->gpio_device, rxi->gpio_bit,
                             txi->gpio_device, txi->gpio_bit,
                             config);
    usart_set_baud_rate(this->usart_device, USART_USE_PCLK, baud);
    usart_enable(this->usart_device);
}

void HardwareSerial::end(void) {
    usart_disable(this->usart_device);
}

/*
 * I/O
 */

int HardwareSerial::read(void) {
	if(usart_data_available(usart_device) > 0) {
		return usart_getc(usart_device);
	} else {
		return -1;
	}
}

int HardwareSerial::available(void) {
    return usart_data_available(this->usart_device);
}

/* Roger Clark. Added function missing from LibMaple code */

int HardwareSerial::peek(void)
{
    return usart_peek(this->usart_device);
}

int HardwareSerial::availableForWrite(void)
{
    return this->usart_device->wb->size-rb_full_count(this->usart_device->wb);
}

size_t HardwareSerial::write(unsigned char ch) {

  BaseType_t sta = xTaskGetSchedulerState();
  int ret = 0;

  if (sta == taskSCHEDULER_RUNNING)
      taskENTER_CRITICAL();

  ret = logger.write(ch);

  if (sta == taskSCHEDULER_RUNNING)
      taskEXIT_CRITICAL();

  // if full?
  if (ret < 0) {

      if (sta == taskSCHEDULER_RUNNING)
          taskENTER_CRITICAL();

      logger.set(BB_BUF_SIZE - 1, 0);
      logger.swap();

      if (sta == taskSCHEDULER_RUNNING)
          taskEXIT_CRITICAL();

      LOG_I(logger.get_read());

      return 1;
  }


  if (ch == '\n') {
      if (sta == taskSCHEDULER_RUNNING)
          taskENTER_CRITICAL();

      logger.write(0);
      logger.swap();

      if (sta == taskSCHEDULER_RUNNING)
          taskEXIT_CRITICAL();

      LOG_I(logger.get_read());
  }

  return 1;
}

size_t HardwareSerial::write_byte(unsigned char ch) {
    usart_putc(this->usart_device, ch);
	return 1;
}

size_t HardwareSerial::write_byte_direct(uint8_t ch) {
  usart_tx_direct(this->usart_device, &ch, 1);
  return 1;
}

/* edogaldo: Waits for the transmission of outgoing serial data to complete (Arduino 1.0 api specs) */
void HardwareSerial::flush(void) {
    while(!rb_is_empty(this->usart_device->wb)); // wait for TX buffer empty
    while(!((this->usart_device->regs->SR) & (1<<USART_SR_TC_BIT))); // wait for TC (Transmission Complete) flag set
}
