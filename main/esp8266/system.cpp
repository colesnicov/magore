/**
 @file system.c

 @brief Memory and system utilities

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

#include "user_config.h"

//#include <stdlib.h>

#include <malloc.h>
#include <esp_system.h>
#include <esp8266/system.hpp>

//#include "mathio.h"

/// @brief calloc may be aliased to safecalloc
//#undef calloc
/// @brief free may be aliased to safefree
//#undef free
/// @brief malloc may be aliased to safecalloc
//#undef malloc

extern void *_heap_start;
//#define HEAP_START  ((uint32_t) & (_heap_start))
//#define HEAP_END    ((uint32_t) (0x3FFFC000UL - 1UL))

/// @brief Return Free memory 
///
/// @return free memory in bytes.
/// @see malloc().
MEMSPACE
size_t freeRam() {
	return ((size_t) esp_get_free_heap_size());
}

/// @brief Display Free memory and regions
/// @return void
MEMSPACE
void PrintRam() {
	printf("Heap Free(%d) bytes\n", esp_get_free_heap_size());
//	printf("Heap Start(%08x), Heap End(%08x), Delta(%d)\n",
//	HEAP_START, HEAP_END, HEAP_END - HEAP_START);
}

/// @brief Safe Calloc -  Display Error message if Calloc fails
///
///  - We check if the pointer was in the heap.
///  - Otherwise it may have been statically defined - display error.
/// @param[in] nmemb: number of elements
/// @param[in] size:  size of elements
/// @return  void.
MEMSPACE
void* safecalloc(size_t nmemb, size_t size) {
	void *p = calloc(nmemb, size);
	if (!p) {
		printf("safecalloc(%d,%d) failed!\n", nmemb, size);
		PrintRam();
	}
	return (p);
}

/// @brief Safe Malloc -  Display Error message if Malloc fails
///
///  - We check if the pointer was in the heap.
///  - Otherwise it may have been statically defined - display error.
/// @param[in] size:  size 
/// @return  void.
MEMSPACE
void* safemalloc(size_t size) {
	return (safecalloc(size, 1));
}

/// @brief Safe free -  Only free a pointer if it is in malloc memory range.
///  We want to try to catch frees of static or bogus data
///
#ifdef ESP8266
/// FIXME HEAP_END is not likely exact 
///  If it is not exact it is larger or equal so the test is usefull
/// @see https://github.com/esp8266/esp8266-wiki/wiki/Memory-Map
#endif
///
///  - We check if the pointer was in the heap.
///  - Otherwise it may have been statically defined - display error.
/// @param[in] p: pointer to free.
/// @return  void.
MEMSPACE
void safefree(void *p) {
//	if ((uint32_t) p >= HEAP_START && (uint32_t) p <= HEAP_END) {
		free(p);
		return;
//	}
	printf("safefree: FREE ERROR (%08x)\n", (uint32_t)p);
	PrintRam();
}

/// @brief reset system
/// @return  void
MEMSPACE
void reset(void) {
	esp_restart();
}

/// @brief reset watchdog
/// @return  void
MEMSPACE
void wdt_reset(void) {
	WRITE_PERI_REG(0x60000914, 0x73);
}
