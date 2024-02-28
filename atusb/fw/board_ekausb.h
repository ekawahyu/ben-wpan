/*
 * fw/board_ekausb.h
 *
 * Written 2023 by Eka Susilo
 * 
 * Based on fw/board_atusb written by Stefan Schmidt
 * Based on fw/board_hulusb written by Filzmaier Josef
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef BOARD_EKAUSB_H
#define	BOARD_EKAUSB_H

#include <stdbool.h>
#include <stdint.h>

#define LED_RED_PORT        B
#define LED_RED_BIT         0
#define LED_PORT            LED_RED_PORT
#define LED_BIT             LED_RED_BIT

#define	nRST_RF_PORT        C
#define	nRST_RF_BIT         6
#define	SLP_TR_PORT         B
#define	SLP_TR_BIT          4

#define SCLK_PORT           D
#define SCLK_BIT            5
#define	MOSI_PORT           D
#define	MOSI_BIT            3

#define	MISO_PORT           D
#define	MISO_BIT            2
#define	nSS_PORT            D
#define	nSS_BIT             1
#define	IRQ_RF_PORT         D
#define	IRQ_RF_BIT          0

#define SR_TX_AUTO_CRC_ON   0x04, 0x20, 5
#define SR_CHANNEL          0x08, 0x1f, 0

#define RG_CC_CTRL_1        (0x14)

#define SPI_WAIT_DONE()	while (!(UCSR1A & 1 << RXC1))
#define SPI_DATA	UDR1

void set_clkm(void);
void board_init(void);

void led_red(bool on);

void spi_begin(void);
void spi_off(void);
void spi_init(void);

#ifdef DEBUG
void printStatus(void);
#define PRINT_STATUS() printStatus()
#endif

#endif /* BOARD_EKAUSB_H */
