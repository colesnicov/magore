/**
 @file ili9341.h

 @brief ili9341 driver inspired by Adafruit ili9341 code
        All code in this file has been rewritten by Mike Gore
 @par Copyright &copy; 2015 Mike Gore, GPL License
 @par You are free to use this code under the terms of GPL
   please retain a copy of this notice in any code you use it in.

 @par Copyright &copy; 2013 Adafruit Industries.  All rights reserved.
 @see https://github.com/adafruit/Adafruit-GFX-Library

 @par Line drawing function CERTS
 @see https://github.com/CHERTS/esp8266-devkit/tree/master/Espressif/examples/esp8266_ili9341


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

#ifndef _ILI9341_H_
#define _ILI9341_H_
#include <stdint.h>

typedef struct
{
    int16_t xpos;       // x pos
    int16_t ypos;       // y pos
    int16_t x;
    int16_t y;
    int16_t w;
    int16_t h;
    uint16_t  font;	// font index
    uint16_t  flags;// font fixed == 1, var == 0
    uint16_t fg;
    uint16_t bg;
    uint8_t rotation;
	uint8_t tabstop;
} window;

///@brief 2D display point - display coordinates are int16
typedef struct {
//	p2_int16_t():X(0),Y(0){};
	int16_t X;
	int16_t Y;
} p2_int16_t;

///@brief 3D display point - display coordinates are int16
typedef struct {
	int16_t X;
	int16_t Y;
	int16_t Z;
} p3_int16_t;


// Named address space
#ifndef MEMSPACE
#define MEMSPACE /**/
#endif

// Weak attribute
#ifndef WEAK_ATR
#define WEAK_ATR __attribute__((weak))
#endif

#include <ili9341/font.hpp>
#include "../../../../main/3rd_party/ili9341_adafruit.hpp"

// FIXED font is default
#define FONT_VAR   1
#define WRAP_H	   2
#define WRAP_V	   4
#define WRAP	   (WRAP_H | WRAP_V)

#define TFT_W (MAX_TFT_X-MIN_TFT_X+1)
#define TFT_H (MAX_TFT_Y-MIN_TFT_Y+1)
#define TFT_XOFF (MIN_TFT_X)
#define TFT_YOFF (MIN_TFT_Y)

/// @brief  Pass 8-bit (each) R,G,B, get back 16-bit packed color
/// ILI9341 defaults to MSB/LSB data so we have to reverse it
/// @param[in] r: red data
/// @param[in] b: blue data
/// @param[in] g: green data
#define tft_RGBto565(r, g, b) ((uint16_t) (((r) & 0xf8) << 8) | (((g) & 0xfc) << 3) | (((b) & 0xf8) >>3))

#define SWAP(a, b) do { a ^= b; b ^= a; a ^= b; } while(0)
#define ABS(x) ((x)<0 ? -(x) : (x))

// ============================================================

/* font.c */
int font_max ( void );
int font_H ( int font );
int font_W ( int font );
int font_attr ( window *win , int c , _fontc *f );

// ============================================================
/* vfont.c */
int16_t sx ( float scale , int16_t xoff , int16_t X );
int16_t sy ( float scale , int16_t yoff , int16_t Y );
void drawSVG ( window *win , int16_t x , int16_t y , int16_t c , float scale , uint16_t color, int16_t fill );

// ============================================================


/* ili9341.c */
void tft_dc_init(void);
void tft_dc_set(int addr);

void tft_Cmd ( uint8_t cmd );
uint8_t tft_Data ( uint8_t data );
void tft_Cmd_Data_TX ( uint8_t cmd , uint8_t *data , int bytes );
int32_t tft_abs_window ( window *win , int16_t x , int16_t y , int16_t w , int16_t h );
int32_t tft_rel_window ( window *win , int16_t x , int16_t y , int16_t w , int16_t h );
MEMSPACE uint32_t tft_readRegister ( uint8_t command , uint8_t parameter );
MEMSPACE uint32_t tft_readId ( void );
void tft_bit_blit ( window *win , uint8_t *ptr , int16_t x , int16_t y , int16_t w , int16_t h );
void tft_fillWin ( window *win , uint16_t color );
void tft_flood ( window *win , int16_t x , int16_t y , uint16_t border , uint16_t fill );
int tft_push_xy ( int16_t x , int16_t y );
int tft_pop_xy ( int16_t *x , int16_t *y );
int tft_floodline ( window *win , int16_t x , int16_t y , uint16_t border , uint16_t fill );
int tft_FillPolyLine ( window *win , int16_t x , int16_t y , int w , uint16_t color );
void tft_fillRectWH ( window *win , int16_t x , int16_t y , int16_t w , int16_t h , uint16_t color );
void tft_fillRectXY ( window *win , int16_t x , int16_t y , int16_t xl , int16_t yl , uint16_t color );
void tft_drawPixel ( window *win , int16_t x , int16_t y , int16_t color );
void tft_writeRect ( window *win , int16_t x , int16_t y , int16_t w , int16_t h , uint16_t *color );
void tft_readRect ( window *win , int16_t x , int16_t y , int16_t w , int16_t h , uint16_t *color );
void tft_Vscroll ( window *win , int dir );
uint16_t tft_readPixel ( window *win , int16_t x , int16_t y );
MEMSPACE void tft_setRotation ( uint8_t m );
void tft_565toRGB ( uint16_t color , uint8_t *r , uint8_t *g , uint8_t *b );
MEMSPACE void tft_invertDisplay ( int flag );
MEMSPACE int tft_window_clip ( window *win );
MEMSPACE void tft_clip_xy ( window *win , int16_t *X , int16_t *Y );
MEMSPACE int tft_window_clip_args ( window *win , int16_t *x , int16_t *y , int16_t *w , int16_t *h );
MEMSPACE void tft_window_init ( window *win , int16_t x , int16_t y , int16_t w , int16_t h );
MEMSPACE void tft_setTextColor ( window *win , uint16_t fg , uint16_t bg );
MEMSPACE void tft_setpos ( window *win , int16_t x , int16_t y );
MEMSPACE void tft_set_textpos ( window *win , int16_t x , int16_t y );
MEMSPACE void tft_set_font ( window *win , uint16_t index );
MEMSPACE void tft_font_fixed ( window *win );
MEMSPACE void tft_font_var ( window *win );
int tft_get_font_height ( window *win );
void tft_drawFastVLine ( window *win , int16_t x , int16_t y , int16_t h , uint16_t color );
void tft_drawFastHLine ( window *win , int16_t x , int16_t y , int16_t w , uint16_t color );
void tft_drawLine ( window *win , int16_t x0 , int16_t y0 , int16_t x1 , int16_t y1 , uint16_t color );
int tft_Bezier2 ( window *win , p2_int16_t S , p2_int16_t C , p2_int16_t T , int steps , uint16_t color );
int tft_Bezier3 ( window *win , p2_int16_t S , p2_int16_t C1 , p2_int16_t C2 , p2_int16_t T , int steps , uint16_t color );
MEMSPACE void tft_cleareol ( window *win );
MEMSPACE void tft_clearline ( window *win );
void tft_putch ( window *win , int c );

// ============================================================
/* ili9341_hal.c */
void tft_spi_init ( uint32_t prescale );
void tft_spi_begin ( void );
void tft_spi_end ( void );
void tft_reset_init ( void );
void tft_reset_enable ( void );
//void tft_addr_init ( void );
void tft_reset_disable ( void );
void tft_spi_TX ( uint8_t *data , int bytes , uint8_t command );
void tft_spi_TXRX ( uint8_t *data , int bytes , uint8_t command );
void tft_spi_RX ( uint8_t *data , int bytes , uint8_t command );
MEMSPACE window *tft_init ( void );

#endif // _ILI9341_SUP_H_
