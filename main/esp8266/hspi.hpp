/*
 * hspi.h
 *
 *  Created on: 12 ���. 2015 �.
 *      Author: Sem
 */

#ifndef INCLUDE_HSPI_H_
#define INCLUDE_HSPI_H_
#include <stdint.h>

#define SPI         0
#define HSPI        1
#define HSPI_FIFO_SIZE  64

#define CONFIG_FOR_TX           1
#define CONFIG_FOR_RX_TX        2

/* hspi.c */
void hspi_init ( uint32_t prescale , int hwcs );
void hspi_waitReady ( void );
void hspi_TX ( uint8_t *data , int count );
void hspi_TXRX ( uint8_t *data , int count );
void hspi_RX ( uint8_t *data , int count );

#endif                                            /* INCLUDE_HSPI_H_ */
