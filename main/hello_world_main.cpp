/**
 @file user_main.c

 @brief Main user and initialization code
  This initialize the platform and runs the main user task.
  All display updates and task are called from here.

 @par Copyright &copy; 2015 Mike Gore, GPL License
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
#include <esp8266/eagle_soc.h>
#include <esp8266/hal.hpp>
#include <esp8266/hspi.hpp>
#include <ili9341/tft_printf.hpp>
#include <lib/matrix.hpp>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <rom/ets_sys.h>
#include <stdio.h>
#include <time.h>
#include <xpt2046/calibrate.h>

#include "esp8266/cpu.h"
#define DISPLAY
#define CIRCLE

#include "user_config.h"





#ifdef ADF4351
	#include "adf4351.h"
#endif

//#include "matrix.h"
#include <esp8266/system.hpp>
//#include "stringsup.h"

#ifdef DISPLAY
	#include <ili9341/ili9341.hpp>


	#ifdef WIRECUBE
		#include "cordic/cordic.h"
		#include "wire.h"
		#include "cube_data.h"
	#endif

	#ifdef EARTH
		#include "cordic/cordic.h"
		#include "wire.h"
		#include "earth_data.h"
	#endif

	#ifdef XPT2046
		#include "xpt2046.h"
		#include "calibrate.h"
		// Calibration status
	#endif

		extern int tft_is_calibrated;
	extern mat_t tft_calX,tft_calY;

	/*
	 * Window layouts    optional
	 *
	 *         wintop    winearth
	 *         winmsg    wincube
	 *         winbottom
	 */

	/* Master Window, Full size of TFT */
	window *master;

	/* Bottom status window */
	window _winbottom;
	window *winbottom = &_winbottom;

	/* wintop and winearth have the same height
	* Top Left status window */
	window _wintop;
	window *wintop = &_wintop;

	/* Earth Top Right status window */
	window _winearth;
	window *winearth = &_winearth;

	/* winmsg and winearth have the same height
	* Middle Left status window */
	window _winmsg;
	window *winmsg = &_winmsg;

	/* Right Wireframe window */
	window _wincube;
	window *wincube = &_wincube;

	// Rotation angle
	 double degree = 0.0;
	// Rotation increment
	 double deg_inc = 4;
	// Scale increment
	 double dscale_inc;
	// Scale factor
	 double dscale;
	// Scale maximum
	 double dscale_max;

	 long count = 0;
	 int rad;
//	 point V;
//	 point S;


#endif


////extern int printf(const char *fmt, ...);
//void ets_timer_disarm(ETSTimer *ptimer);
//void ets_timer_setfn(ETSTimer *ptimer, ETSTimerFunc *pfunction, void *parg);

//unsigned long ms_time = 0;

// ==================================================================
// ==================================================================

/// @brief  Clear 1000HZ timer
/// We loop in case the update of ms_time is not "atomic" - done in a single instruction
/// @return  void.
//MEMSPACE
//void ms_clear()
//{
//	while(ms_time)
//		ms_time = 0;
//}

/// @brief  Read 1000HZ timer
/// We loop in case the update of ms_time is not "atomic" - done in a single instruction
/// @return  time in milliseconds
//MEMSPACE
//unsigned long ms_read()
//{
//	unsigned long ret = 0;
//	while(ret != ms_time)
//		ret = ms_time;
//	return(ret);
//}

/**
 @brief 1000HZ timer task
 @return void
*/
//void ms_task(void)
//{
//    ms_time++;
//}

/// @brief  Initialize 1000HZ timer task
/// @return  void.
//MEMSPACE
//void ms_init()
//{
//	ms_time = 0;
//    if(set_timers(ms_task,1) == -1)
//        printf("Clock task init failed\n");
//}


// FIXME we need to add time zone and timezone processing rules
//int ntp_init = 0;
//void ntp_setup(void)
//{
//    tv_t tv;
//    tz_t tz;
//	time_t sec;
//	struct ip_info getinfo;
//
//
//	// Wait until we have an IP address before we set the time
//    if(!network_init)
//		return;
//
//	if(ntp_init == 0)
//    {
//        ip_addr_t *addr = (ip_addr_t *)safecalloc(sizeof(ip_addr_t),1);
//
//		// form pool.ntp.org
//		ipaddr_aton("206.108.0.131", addr);
//		sntp_setserver(1,addr);
//		ipaddr_aton("167.114.204.238", addr);
//		sntp_setserver(2,addr);
//
//#if 0
//		// Alternate time setting if the local router does NTP
//		if(wifi_get_ip_info(0, &getinfo))
//		{
//			printf("NTP:0 GW: %s\n", ipv4_2str(getinfo.gw.addr));
//			printf("NTP:0 IP: %s\n", ipv4_2str(getinfo.ip.addr));
//			sntp_setserver(1, & getinfo.gw);
//			sntp_setserver(2, & getinfo.ip);
//		}
//		else
//		{
//			printf("NTP:0 failed to get GW address\n");
//			return;
//		}
//#endif
//
//        if( sntp_set_timezone(0) )
//		{
//			printf("NTP: set_timeone OK\n");
//			sntp_init();
//            safefree(addr);
//		    ntp_init = 1;
//            printf("NTP:1\n");
//		}
//		else
//		{
//			printf("NTP: set_timeone Failed\n");
//		}
//    }
//
//	if(ntp_init == 1)
//	{
//		// they hard coded it to +8 hours from GMT
//		if( (sec = sntp_get_current_timestamp()) > 10 )
//		{
//			sntp_stop();
//			ntp_init = 2;
//		}
//	}
//	if(ntp_init == 2)
//	{
//		time_t s;
//
//		tm_t *p;
//
//		printf("NTP:2\n");
//
//		// they return GMT + 8
//        // sec = sec - (8UL * 3600UL);
//
//        tv.tv_sec = sec;
//		printf("ntp_init: %s\n", asctime(gmtime(&sec)));
//		printf("ntp_init: %s\n", ctime_gm(&sec));
//
//        tv.tv_usec = 0;
//        tz.tz_minuteswest = 300;
//		tz.tz_dsttime = 0;
//
//        settimeofday(&tv, &tz);
//
//        printf("SEC:%ld\n",sec);
//        printf("TIME:%s\n", ctime(&sec));
//		printf("Zone: %d\n", (int) sntp_get_timezone());
//		ntp_init = 3;
//
//		set_dst(tv.tv_sec);
//
//		print_dst_gmt();
//		print_dst();
//
//		p = gmtime(&tv.tv_sec);
//		mktime(p);
//		printf("Localtime: %s\n", asctime(p));
//    }
//}

void user_tasks()
{
	char buffer[260];
	int argc;
	char *argv[10];


}
// Signal strength update interval
int signal_loop = 0;

time_t seconds = 0;

/**
 @brief test task
  Runs corrected cube demo from Sem
  Optionally wireframe Earth viewer
 @return void
*/

//extern uint8_t ip_msg[];

int skip = 0;

long last_time10 = 0;
long last_time50 = 0;
time_t sec = 0;

int loop_cnt = 0;

// VCC voltage divider
#define R1 330000.0
// R2 is actually a variable resister
#define R2 100000.0

// ADC read error scale
// Measured error
#define VERROR 0.95
// This is reduced to a single constant by the compiler
#define VSCALE (VERROR*((R1+R2)/R2)/1024.0)

/**
 @brief return system_adc_read scaled to a float
  T_OUT pin is connected to the junction of a voltage divider R1 and R2
  R1 is connected to VCC
  R2 is connected to ground
  T_OUT is connected to the junction of R1 and R2
//FIXME T_OUT has a loading value
 @return float
*/
//float adc_read()
//{
//	uint16_t system_adc_read(void);
//	// system adc read returns 0 .. 1023
//	// range 0 .. 1.0V
//	return( ((float)system_adc_read()) * VSCALE );
//}

// working on vector fonts

// ============================================================


// ============================================================

// main task loop called by yield code
void user_loop(void)
{
	extern int connections;
	uint32_t time1,time2;
	long t;
#ifdef DISPLAY
	char time_tmp[32];
	uint8_t red, blue,green;
	int touched;
	uint16_t X,Y;
#endif

	// getinfo.ip.addr, getinfo.gw.addr, getinfo.netmask.addr
//	struct ip_info getinfo;
	char *ptr;

	// ========================================================
	// Run all remaining tasks once every 1mS
//	t = ms_read();
//	if((t - last_time10) < 1U)
//		return;
//	last_time10 = t;
	// ========================================================
// Tasks that must run very fast , once every millisecond should be at the top

#ifdef ADF4351
	ADF4351_task();
#endif
#ifdef XPT2046
	XPT2046_task();
#endif

	// ========================================================
	// Only run every 50mS
//	t = ms_read();
//	if((t - last_time50) < 50U)
//		return;
//	last_time50 = t;
	// ========================================================

	user_tasks();

	// NTP state machine
//	ntp_setup();


#ifdef DISPLAY
	#ifdef XPT2046
		if(tft_is_calibrated)
		{
			touched = tft_touch_key(master,(uint16_t *)&X, (uint16_t *)&Y);
			#if XPT2046_DEBUG
				if(touched)
					tft_printf(winmsg,"X:%d,Y:%d\n",(int)X,(int)Y);
			#endif
		}
	#endif

	#ifdef NETWORK_TEST
		servertest_message(winmsg);
	#endif

	#ifdef DEBUG_STATS
		#ifdef VOLTAGE_TEST
			#ifdef DEBUG_STATS
				// Do NOT run adc_read() every millisecond as system_adc_read() blocks WIFI
				tft_set_textpos(wintop, 0,2);
				tft_printf(wintop,"Volt:%2.2f\n", (float)adc_read());
			#endif
		#endif // VOLTAGE_TEST

		count += 1;
		tft_set_textpos(wintop, 0,0);
		tft_set_font(wintop,0);
		tft_font_fixed(wintop);
		tft_printf(wintop,"Iter:% 10ld, %+7.2f\n", count, degree);
	#endif

	#ifdef CIRCLE
		rad = dscale; // +/- 90
		tft_drawCircle(wincube, wincube->w/2, wincube->h/2, rad ,wincube->bg);
		// RGB
	#endif

	// reset cube to background
	#ifdef WIRECUBE
		V.x = degree;
		V.y = degree;
		V.z = degree;
		// Cube points were defined with sides of 1.0
		// We want a scale of +/- w/2
		wire_draw(wincube, cube_points, cube_edges, &V, wincube->w/2, wincube->h/2, dscale, wincube->bg);
	#endif

	degree += deg_inc;
	dscale += dscale_inc;

	if(degree <= -360)
		deg_inc = 4;
	if(degree >= 360)
		deg_inc = -4;

	if(dscale < dscale_max/2)
	{
	   dscale_inc = -dscale_inc;
	}
	if(dscale > dscale_max)
	{
	   dscale_inc = -dscale_inc;
	}

	#ifdef WIRECUBE
		V.x = degree;
		V.y = degree;
		V.z = degree;
		wire_draw(wincube, cube_points, cube_edges, &V, wincube->w/2, wincube->h/2, dscale, ILI9341_WHITE);
	#endif

	#ifdef CIRCLE
		// Display bounding circle that changes color around the cube
		if(dscale_inc < 0.0)
		{
			red = 255;
			blue = 0;
			green = 0;
		}
		else
		{
			red = 0;
			blue = 0;
			green = 255;
		}
		rad = dscale; // +/- 90
		tft_drawCircle(wincube, wincube->w/2, wincube->h/2, rad, tft_RGBto565(red,green,blue));
	#endif
#endif	// DISPLAY

	// ========================================================
	// Tasks run only once every second go after this
	time(&sec);
	if(sec == seconds)
		return;
	seconds=sec;
	// ========================================================

#ifdef DISPLAY
	// ========================================================
	// TIME
	tft_set_textpos(winbottom, 0,0);
	//Tue May 17 18:56:01 2016
	strncpy(time_tmp,ctime(&sec),31);
	time_tmp[19] = 0;
	tft_printf(winbottom," %s", time_tmp);
	tft_cleareol(winbottom);
	tft_set_textpos(winbottom, 0,1);

	// ========================================================
	// CONNECTION status
	//tft_printf(winbottom," %s", ip_msg);
	// IP and disconnected connection state only
//	if(wifi_get_ip_info(0, &getinfo))
//		tft_printf(winbottom," %s", ipv4_2str(getinfo.ip.addr));
//	else
		tft_printf(winbottom," Disconnected");
	tft_cleareol(winbottom);

	#ifdef DEBUG_STATS
		// ========================================================
		// HEAP size
		tft_set_textpos(wintop, 0,1);
		tft_printf(wintop,"Heap: %d, Conn:%d\n",
		system_get_free_heap_size(), connections);

		// ========================================================
		// WIFI status
		tft_set_textpos(wintop, 0,3);
		tft_printf(wintop,"CH:%02d, DB:%+02d\n",
		wifi_get_channel(),
		wifi_station_get_rssi());
	#endif	// DEBUG_STATS
#endif	//DISPLAY

}

int inloop = 0;
void loop()
{
	int ret;
	if(inloop)
	{
		printf("Error: loop() task overrun\n");
		inloop = 0;
		return;
	}
	inloop = 1;

	// ========================================================
	// We should not have any SPI devices enabled at this point
	ret = spi_chip_select_status();
	if(ret != 0xff)
	{
		printf("Error: loop() entered with spi_cs = %d\n",ret);
		spi_end_transaction(ret);
		return;
	}


	user_loop();

	// We should not have any SPI devices enabled at this point
	ret = spi_chip_select_status();
	if(ret != 0xff)
	{
		printf("Error: loop() entered with spi_cs = %d\n",ret);
		spi_end_transaction(ret);
		return;
	}

	inloop = 0;
}


#ifdef DISPLAY
	#if ILI9341_DEBUG & 1
	MEMSPACE
	void read_tests(window *win)
	{
		int x,y;
		uint16_t color;
		uint16_t buffer[3*16];	// 16 RGB sets
		y = 4;
		tft_readRect(win, x, y, 16, 1, buffer);
		for(x=0;x<16;++x)
		{
			printf("x:%d,y:%d,c:%04x\n", x,y,buffer[x]);
		}
		 printf("\n");
	}
	#endif


#endif	//DISPLAY


void user_help()
{
	#ifdef POSIX_TESTS
		posix_help();
	#endif
	#ifdef FATFS_TESTS
		fatfs_help(1);
	#endif
	#ifdef ADF4351
		adf4351_help();
	#endif
	printf(
		"help\n"
        "connection\n"
        "calibrate N\n"
        "calibrate_test N\n"
		"display_clock\n"
        "draw C[1]\n"
        "mem\n"
		"pixel\n"
        "rotate N\n"
		"setdate YYYY MM DD HH:MM:SS\n"
		"time\n"
		"timetest\n"
		"\n");
}



/// @brief help functions test parser
///
/// - Keywords and arguments are matched against test functions
/// If there are matched the function along with its arguments are called.

/// @param[in] str: User supplied command line
///
/// @return 1 The return code indicates a command matched.
/// @return 0 if no rules matched


#ifdef TEST_FLASH
	ICACHE_RODATA_ATTR uint8_t xxx[] = { 1,2,3,4,5,6,7,8 };
	/// @brief Test flash read code
	/// @param[in] *win: window structure
	/// @return  void
	MEMSPACE
	void test_flashio(window *win)
	{
		uint16_t xpros,ypos;
		for(i=0;i<8;++i)
		{
			printf("%02x", read_flash8((uint32_t) &xxx+i));
		}
		printf("\n");
		printf("%08x, %08x", read_flash32((uint8_t *)&xxx), read_flash16((uint8_t *)&xxx));
	}
#endif

/**
 test byte order and basic type sizes
*/
void test_types()
{
	int i;
// Test byte order
// extensa has LSB to MSB byte order LITTLE_ENDIAN
	union UUU
	{
	  unsigned int  wide;
	  unsigned char byte8[sizeof(unsigned int)];
	};
	volatile union UUU u;

	printf("Byte Order of 0x12345678:\n");
	u.wide = 0x12345678;
	for (i=0; i < sizeof(unsigned int); i++)
		printf("byte[%d] = %02x\n", i, u.byte8[i]);

// Test basic type sizes
    printf("sizeof (double) = %d\n", sizeof (double ) );
    printf("sizeof (float) = %d\n", sizeof (float ) );
    printf("sizeof (long long) = %d\n", sizeof (long long ) );
    printf("sizeof (long) = %d\n", sizeof (long ) );
    printf("sizeof (int) = %d\n", sizeof (int ) );
    printf("sizeof (char) = %d\n", sizeof (char ) );

    printf("long: %ld\n", 123456789L);
    printf("unsigned long: %lu\n", 123456789L);
    printf("long hex: %lx\n", 123456789L);

//    printf("int : %d\n", 123456789L);
//    printf("unsigned int: %u\n", 123456789L);
    printf("int hex: %lx\n", 123456789L);

    printf("int: %d\n", 12345);
    printf("unsigned int: %u\n", 12345);
    printf("int hex: %x\n", 12345);

}

#ifdef DISPLAY
void setup_windows(int rotation, int debug)
{
	int16_t x,y,w,h;
	uint32_t ID;
	extern uint16_t tft_ID;
	ID = tft_ID;

	// Set master rotation
	tft_setRotation(rotation);
	tft_setTextColor(master, ILI9341_WHITE,ILI9341_BLUE);
	tft_fillWin(master, master->bg);

#if ILI9341_DEBUG & 1
		if(debug)
			printf("\nDisplay ID=%08lx\n",ID);
#endif

	// Message window setup
#ifdef EARTH
		w = master->w * 7 / 10;
#else
		w = master->w;
#endif
		// TOP
#ifdef DEBUG_STATS
		tft_window_init(wintop,0,0, w, font_H(0)*4);
		tft_setTextColor(wintop, ILI9341_WHITE, ILI9341_NAVY);
#else
		tft_window_init(wintop,0,0, w, font_H(2)*2);
		tft_setTextColor(wintop, ILI9341_WHITE, tft_RGBto565(0,64,255));
#endif
	tft_set_font(wintop,0);
	tft_font_var(wintop);
	tft_fillWin(wintop, wintop->bg);
	tft_set_textpos(wintop, 0,0);

#ifdef EARTH
		tft_window_init(winearth,w,0, master->w - w + 1, wintop->h);
		tft_setTextColor(winearth, ILI9341_WHITE, ILI9341_NAVY);
		tft_fillWin(winearth, winearth->bg);
#endif

	// BOTOM
	// TIME,DATE
	tft_window_init(winbottom, 0, master->h - 1 - font_H(2)*2,
		master->w, font_H(2)*2);
	if(master->rotation & 1)
		tft_set_font(winbottom,2);
	else
		tft_set_font(winbottom,1);
	tft_font_var(winbottom);
	tft_setTextColor(winbottom, 0, tft_RGBto565(0,255,0));
	tft_fillWin(winbottom, winbottom->bg);
	tft_set_textpos(winbottom, 0,0);

	// Message window setup
#ifdef WIRECUBE
		w = master->w * 7 / 10;
#else
		w = master->w;
#endif

	// MSG
	tft_window_init(winmsg,0,wintop->h,
			w, master->h - (wintop->h + winbottom->h));

	tft_setTextColor(winmsg, ILI9341_WHITE,ILI9341_BLUE);
	tft_fillWin(winmsg, winmsg->bg);
	// write some text
	tft_set_font(winmsg,0);
	tft_font_var(winmsg);
	tft_set_textpos(winmsg, 0,0);

	// CUBE setup
#ifdef WIRECUBE
		/* Setup cube/wireframe demo window */
		/* This is to the right of the winmsg window and the same height */
		tft_window_init(wincube, winmsg->w, wintop->h, master->w - winmsg->w, winmsg->h);
		tft_setTextColor(wincube, ILI9341_WHITE,ILI9341_BLUE);
		tft_fillWin(wincube, wincube->bg);
#endif

#ifdef DEBUG_STATS
	if(debug)
	{
		// Display ID
		tft_setTextColor(winmsg, ILI9341_RED,winmsg->bg);
		tft_printf(winmsg, "DISP ID: %04lx\n", ID);
		tft_setTextColor(winmsg, ILI9341_WHITE,winmsg->bg);
#ifdef XPT2046
			if(!tft_is_calibrated)
			{
				tft_set_font(winmsg,0);
				tft_printf(winmsg,"Please Calibrate Display\n");
				tft_printf(winmsg,"serial command: calibrate N\n");
				tft_printf(winmsg,"N is rotation, 0..3\n");
				tft_printf(winmsg,"%d is current\n",master->rotation);
			}
#endif
	}
#endif


	// Cube points were defined with sides of 1.0
	// We want a scale of +/- w/2
#ifdef WIRECUBE
		if(wincube->w < wincube->h)
			dscale_max = wincube->w/2;
		else
			dscale_max = wincube->h/2;

		dscale = dscale_max;
		dscale_inc = dscale_max / 100;
#endif

#if ILI9341_DEBUG & 1
	if(debug)
	{
		printf("Test Display Read\n");
		read_tests(winmsg);
	}
#endif

	// Draw Wireframe earth in message area
#ifdef EARTH
	// Earth points were defined with radius of 0.5, diameter of 1.0
	// We want a scale of +/- w/2
		double tscale_max;
		if(winearth->w < winearth->h)
			tscale_max = winearth->w;
		else
			tscale_max = winearth->h;
		V.x = -90;
		V.y = -90;
		V.z = -90;
		// draw earth
	// Earth points were defined over with a scale of -0.5/+0.5 scale - so scale must be 1 or less
		wire_draw(winearth, earth_data, NULL, &V, winearth->w/2, winearth->h/2, tscale_max, winearth->fg);
#endif

}
#endif	//DISPLAY

/**
 @brief main() Initialize user task
 @return void
*/

extern "C" void  app_main(void)
{
	int i;
    char time[20];
	int ret;
	uint16_t *ptr;
	double ang;
//	extern web_init();
	int w,h;

//	ip_msg[0] = 0;

// CPU
// 160MHZ
   REG_SET_BIT(0x3ff00014, BIT(0));
// 80MHZ
//   REG_CLR_BIT(0x3ff00014, BIT(0));

	os_delay_us(200000L);	// Power Up dalay - lets power supplies and devices settle

	// Configure the UART
	//uart_init(BIT_RATE_115200,BIT_RATE_115200);
//	uart_init(BIT_RATE_74880,BIT_RATE_74880);

	os_delay_us(200000L);	// Power Up dalay - lets power supplies and devices settle
	os_delay_us(200000L);	// Power Up dalay - lets power supplies and devices settle
	printf("\n\n\n\n");
    sep();
	printf("System init...\n");
    printf("ESP8266 multidevice project\n");
    printf(" (c) 2014-2017 by Mike Gore\n");
    printf(" GNU version 3\n");
    printf("-> https://github.com/magore/esp8266_ili9341\n");
//    printf("   GIT last pushed:   %s\n", GIT_VERSION);
//    printf("   Last updated file: %s\n", LOCAL_MOD);


    sep();
    PrintRam();

    sep();
	printf("HSPI init...\n");
	spi_init_bus();
	hspi_init(1,0);

	printf("Timers init...\n");
//	init_timers();

	// 1000HZ timer
//	ms_init();

	test_types();

	// Functions manage user defined address pins
	tft_dc_init();

//	initialize_clock(300);



#ifdef DISPLAY
    sep();
	#ifdef ILI9341_CS
		chip_select_init(ILI9341_CS);
	#endif

	#ifdef XPT2046_CS
		XPT2046_spi_init();
	#endif

	// Initialize TFT
	master = tft_init();

 	tft_calX = MatRead("/tft_calX");
 	tft_calY = MatRead("/tft_calY");
	if(tft_calX.data == NULL || tft_calY.data == NULL)
		tft_is_calibrated = 0;
	else
		tft_is_calibrated = 1;
	printf("TFT calibration %s\n", tft_is_calibrated ?  "YES" : "NO");

	// rotateion = 1, debug = 1
	setup_windows(1,1);
#endif

//	wdt_reset();
    sep();
	printf("Setup Tasks\n");

    sep();
//	setup_networking();

	#ifdef TELNET_SERIAL
		printf("Setup Network Serial Bridge\n");
		bridge_task_init(23);
	#endif

//	if ( espconn_tcp_set_max_con(MAX_CONNECTIONS+1) )
//		printf("espconn_tcp_set_max_con(%d) != (%d) - failed!\n",
//			MAX_CONNECTIONS+1, espconn_tcp_get_max_con());
//	else
//		printf("espconn_tcp_set_max_con(%d) = (%d) - success!\n",
//			MAX_CONNECTIONS+1, espconn_tcp_get_max_con());

#ifdef NETWORK_TEST
	printf("Setup Network TFT Display Client\n");
	servertest_setup(TCP_PORT);
#endif

#ifdef WEBSERVER
	printf("Setup Network WEB SERVER\n");
	web_init(80);
#endif

    sep();
    PrintRam();

//	system_set_os_print(0);

} //setup()
