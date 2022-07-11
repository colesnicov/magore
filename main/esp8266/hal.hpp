/**
 @file hal.c
 @brief HAL layer
 SPI, GPIO, address and chip select
 @par Copyright &copy; 2015 Mike Gore, GPL License
 @par You are free to use this code under the terms of GPL
 Please retain a copy of this notice in any code you use it in.

 This is free software: you can redistribute it and/or modify it under the
 terms of the GNU General Public License as published by the Free Software
 Foundation, either version 3 of the License, or (at your option)
 any later version.

 This software is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _INCLUDE_HAL_H_
#define _INCLUDE_HAL_H_
#include <stdbool.h>
#include <stdint.h>

// not defined in eagle_soc.h
#define FUNC_GPIO6 3
#define FUNC_GPIO7 3
#define FUNC_GPIO8 3
#define FUNC_GPIO11 3

/// @brief program SFR to permit normal GPIO mode
#define GPIO_PIN_MODE(pin)       gpio_pin_sfr_mode(pin);

/// @brief program input mode
#define GPIO_PIN_DIR_IN(pin)     GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, 1<<(pin))
#define GPIO_PIN_TST(pin)         BIT_TST(GPIO_REG_READ(GPIO_IN_ADDRESS),(pin))
/// @brief program input mode and read
#define GPIO_PIN_RD(pin)         GPIO_PIN_DIR_IN(pin), GPIO_PIN_TST(pin)

/// @brief program output latch
#define GPIO_PIN_LATCH_LOW(pin)  GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 1<<(pin))
#define GPIO_PIN_LATCH_HI(pin)   GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 1<<(pin))
/// @brief read output latch
#define GPIO_PIN_LATCH_RD(pin)   BIT_TST(GPIO_REG_READ(GPIO_OUT_DATA),(pin))

/// @brief program output mode
#define GPIO_PIN_DIR_OUT(pin)    GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, 1<<(pin))
#define GPIO_PIN_HI(pin)         GPIO_PIN_LATCH_HI(pin),  GPIO_PIN_DIR_OUT(pin)
#define GPIO_PIN_LOW(pin)        GPIO_PIN_LATCH_LOW(pin), GPIO_PIN_DIR_OUT(pin)
#define GPIO_PIN_WR(pin,val)    (val) ? GPIO_PIN_HI(pin) : GPIO_PIN_LOW(pin)

// =============================================
/// @brief GPIO16 pin control
// GPIO pin 16 is special - it uses the RTC GPIO pin
/// @brief program SFR to permit normal GPIO mode
#define GPIO16_PIN_MODE()      gpio_pin_sfr_mode(16)

/// @brief program input mode
#define GPIO16_PIN_DIR_IN()    gpio16_pin_dir(0)
#define GPIO16_PIN_RD()    ( READ_PERI_REG(RTC_GPIO_IN_DATA) & 1 )
/// @brief program input mode and read
#define GPIO16_RD()        ( GPIO16_PIN_DIR_IN(), GPIO16_PIN_RD() )

/// @brief program output latch
#define GPIO16_PIN_LATCH_LOW() \
	WRITE_PERI_REG(RTC_GPIO_OUT, (READ_PERI_REG(RTC_GPIO_OUT)&(uint32_t)0xfffffffe) )
#define GPIO16_PIN_LATCH_HI()  \
	WRITE_PERI_REG(RTC_GPIO_OUT, (READ_PERI_REG(RTC_GPIO_OUT)&(uint32_t)0xfffffffe)|(uint32_t)1)
#define GPIO16_LATCH_RD()  ( READ_PERI_REG(RTC_GPIO_OUT) & 1 )

/// @brief program output mode
#define GPIO16_PIN_DIR_OUT()   gpio16_pin_dir(1)
#define GPIO16_PIN_HI()        GPIO16_PIN_LATCH_HI(), GPIO16_PIN_DIR_OUT()
#define GPIO16_PIN_LOW()       GPIO16_PIN_LATCH_LOW(), GPIO16_PIN_DIR_OUT()
#define GPIO16_PIN_OUT(val)    (val) ? GPIO16_PIN_HI() : GPIO16_PIN_LOW()

//FIXME do pull up modes
#define GPIO16_PIN_FLOAT()     GPIO16_PIN_DIR_IN()

// =============================================

// =============================================

/* hal.c */
void gpio_pin_sfr_mode(int pin);
void gpio16_pin_dir(uint8_t out);
void gpio_pin_out(uint8_t pin, uint8_t val);
uint8_t gpio_pin_rd(uint8_t pin);
void chip_select_init(uint8_t pin);
void chip_select(uint8_t pin);
void chip_deselect(uint8_t pin);
void spi_waitReady(void);
bool spi_init_bus();
void spi_init(uint32_t clock, int pin);
void spi_begin_transaction(uint32_t clock, int pin);
void spi_end_transaction(uint8_t pin);
uint8_t spi_chip_select_status(void);
void spi_TX_buffer(const uint8_t *data, int count);
void spi_RX_buffer(const uint8_t *data, int count);
void spi_TXRX_buffer(const uint8_t *data, int count);
uint8_t spi_RX(void);
void spi_TX(uint8_t data);
uint8_t spi_TXRX(uint8_t data);

#endif    /* _INCLUDE_HAL_H_ */
