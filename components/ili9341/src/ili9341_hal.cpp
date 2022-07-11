/**
 @file ili9341_hal.c

 @brief ili9341 driver inspired by Adafruit ili9341 code
 All code in this file has been rewritten by Mike Gore
 @par Copyright &copy; 2016 Mike Gore, GPL License
 @par You are free to use this code under the terms of GPL
 please retain a copy of this notice in any code you use it in.

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
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/portmacro.h>
#include <sdkconfig.h>
#include <esp_log.h>
#include <esp8266/hal.hpp>
#include <ili9341/ili9341.hpp>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>


// TFT master window definition
extern window tftwin;
extern window *tft;

/// =============================================================
/// =============================================================
/// Start SPI Hardware Abstraction Layer
/// Keep all hardware dependent SPI code in this section

/// @brief cahce of SPI clock devisor
uint32_t tft_clock = -1;

uint16_t tft_ID;

#ifndef CONFIG_ILI9341_CS
#error CONFIG_ILI9341_CS is undefined
#endif

#define tft_delay_us(a) vTaskDelay(pdMS_TO_TICKS(a/1000))

#ifndef CONFIG_ILI9341_CS
#error CONFIG_ILI9341_CS is undefined
#endif

#define TFT_DATA        tft_dc_set(1)
#define TFT_COMMAND     tft_dc_set(0)


// =============================================
/// @brief ADDRESS select HAL
/**
 @brief initialize GPIO pin as address lines for a device
 @return void
 */
void tft_dc_init() {

#if CONFIG_ILI9341_DC == 16
	GPIO16_PIN_LOW();
	GPIO16_PIN_MODE();
#else
	GPIO_PIN_LOW(CONFIG_ILI9341_DC);
	GPIO_PIN_MODE(CONFIG_ILI9341_DC);
#endif

	ESP_LOGD(__FILE__, "TFT init... pin D/C %d", CONFIG_ILI9341_DC);
}

/**
 @brief set address on GPIO lines
 @see chip_addr_init
 @param[in] addr: device address
 @return void
 */
void tft_dc_set(int addr) {

#if CONFIG_ILI9341_DC == 16
	if (addr & 1) {
		GPIO16_PIN_HI();
	} else {
		GPIO16_PIN_LOW();
	}
#else
	if (addr & 1) {
		GPIO_PIN_HI(CONFIG_ILI9341_DC);
	} else {
		GPIO_PIN_LOW(CONFIG_ILI9341_DC);
	}
#endif

}
/// @brief  Initialize TFT SPI clock speed and pin for slow speed 
/// Only used to read the CHIP ID
/// return: void
void tft_spi_init_slow() {
	tft_clock = 2;
	chip_select_init(CONFIG_ILI9341_CS);
}

/// @brief  Initialize TFT SPI clock speed and pin for normal speed 
/// return: void
void tft_spi_init_fast() {
	tft_clock = 1;
	chip_select_init(CONFIG_ILI9341_CS);
}

/// @brief  Obtain SPI bus for TFT display, assert chip select
/// return: void
void tft_spi_begin() {
	spi_begin_transaction(tft_clock, CONFIG_ILI9341_CS);
}

/// @brief  Release SPI bus from TFT display, deassert chip select
/// return: void
void tft_spi_end() {
	spi_end_transaction(CONFIG_ILI9341_CS);
}

/// @brief  Initialize ILI9341 reset GPIO
/// return: void
void tft_reset_init() {
#if defined(CONFIG_ILI9341_RESET) && CONFIG_ILI9341_RESET >= 0
	chip_select_init(CONFIG_ILI9341_RESET);
#endif
}

/// @brief  enable ILI9341 reset 
/// return: void
void tft_reset_enable() {
#if defined(CONFIG_ILI9341_RESET) && CONFIG_ILI9341_RESET >= 0
	chip_enable(CONFIG_ILI9341_RESET);
#endif
}

/// @brief  Initialize ILI9341 command/data GPIO
/// return: void
//void tft_addr_init() {
//	tft_dc_init();
//}

/// @brief  disnable ILI9341 reset 
/// return: void
void tft_reset_disable() {
#if defined(CONFIG_ILI9341_RESET) && CONFIG_ILI9341_RESET >= 0
	chip_disable(CONFIG_ILI9341_RESET);
#endif
}

/// @brief  Transmit 8 bit data array
/// @param[in] *data: data buffer to send 
/// @param[in] bytes: data buffer size
/// @param[in] command: 1 = command, 0 = data 
/// return: void 
void tft_spi_TX(uint8_t *data, int bytes, uint8_t command) {
	spi_waitReady();
	if (command)
		TFT_COMMAND;
	else
		TFT_DATA;
	spi_TX_buffer(data, bytes);
}

/// @brief  Transmit and read 8 bit data array 
/// @param[in] *data: data buffer to send 
/// @param[in] bytes: data buffer size
/// @param[in] command: 1 = command, 0 = data 
/// return: void 
void tft_spi_TXRX(uint8_t *data, int bytes, uint8_t command) {
	spi_waitReady();
	if (command)
		TFT_COMMAND;
	else
		TFT_DATA;
	spi_TXRX_buffer(data, bytes);
}

/// @brief  read 8 bit data array 
/// @param[in] *data: data buffer to send 
/// @param[in] bytes: data buffer size
/// @param[in] command: 1 = command, 0 = data 
/// return: void 
void tft_spi_RX(uint8_t *data, int bytes, uint8_t command) {
	spi_waitReady();
	if (command)
		TFT_COMMAND;
	else
		TFT_DATA;
	spi_RX_buffer(data, bytes);
}

/// @brief Initialize TFT
/// @return diplay ID 9341
MEMSPACE
window* tft_init(void) {
	// Start with slow clock so tft_readId works
	// This is the only function that fails at less then 1.
	// tft_readId is the ONLY SPI bus command that needs this.
	// Nomal reads work fine.
	tft_dc_init();
	tft_spi_init_slow();

	// reset display
	tft_reset_init();
	tft_reset_enable();
	tft_delay_us(10000);
	tft_reset_disable();
	tft_delay_us(1000);

	/* Adafruit 9341 TFT Display Initialization */
	tft_configRegister();

	/* Read the TFT ID value */
	tft_ID = tft_readId();

	// fast SPI
	tft_spi_init_fast();

	/* Setup the master window */
	tft_window_init(tft, TFT_XOFF, TFT_YOFF, TFT_W, TFT_H);
	tft_setRotation(0);
	tft_fillWin(tft, tft->bg);

	return (tft);
}

/// End of SPI HAL interface
/// =============================================================
/// =============================================================
