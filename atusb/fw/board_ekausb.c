/*
 * fw/board_ekausb.c
 *
 * Written 2023 by Eka Susilo
 * Based on fw/board_hulusb written by Filzmaier Josef
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */


#include <stdbool.h>
#include <stdint.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/boot.h>

#define F_CPU   8000000UL
#include <util/delay.h>

#include "usb.h"
#include "at86rf230.h"
#include "board.h"
#include "spi.h"
#include "usb/usb.h"

static bool spi_initialized = 0;

void led_blink8(uint8_t value);

void reset_rf(void)
{
	// volatile uint8_t reg;
	/* set up all the outputs; default port value is 0 */

	DDRB = 0;
	DDRC = 0;
	DDRD = 0;
	PORTB = 0;
	PORTC = 0;
	PORTD = 0;

	OUT(LED_RED);
	SET(LED_RED);
	OUT(nRST_RF);   /* this also resets the transceiver */
	OUT(SLP_TR);

	spi_init();

	/* AT86RF212 data sheet, Appendix B, p166 Power-On Reset procedure */
	/*-----------------------------------------------------------------*/
	CLR(SLP_TR);
	SET(nRST_RF);
	SET(nSS);
	_delay_us(400);

	CLR(nRST_RF);
	_delay_us(2);
	SET(nRST_RF);

	/* 5.1.4.5: Wait t10: 625 ns (min) */

	_delay_us(2);

	// reg = reg;
	// reg = reg_read(REG_PART_NUM);

	reg_write(REG_TRX_CTRL_0, 0x19);

	change_state(TRX_CMD_FORCE_TRX_OFF);
	/*-----------------------------------------------------------------*/

	/* we must restore TRX_CTRL_0 after each reset (7.7.4) */

	set_clkm();
}

void led_red(bool on) {
	if (on)
		CLR(LED_RED);
	else
		SET(LED_RED);
}

void led(bool on)
{
	led_red(on);
}

void led_blink8(uint8_t value)
{
	uint8_t bit = 0x80;

	while(bit) {
		if (value & bit) {
			led(1);
			_delay_ms(500);
			led(0);
			_delay_ms(500);
		}
		else {
			led(1);
			_delay_ms(100);
			led(0);
			_delay_ms(500);
		}
		bit = bit >> 1;
	}
}

void set_clkm(void)
{
	/* CLKM is not connected on EKAUSB and therefore it is running in
	 * async mode. */
	reg_write(REG_TRX_CTRL_0, 0x00);

	/* TX_AUTO_CRC_ON, default disabled */
	subreg_write(SR_TX_AUTO_CRC_ON, 1);
}

void board_init(void)
{
	/* Disable the watchdog timer */

	MCUSR = 0;		/* Remove override */
	WDTCSR |= 1 << WDCE;	/* Enable change */
	WDTCSR = 1 << WDCE;	/* Disable watchdog while still enabling
				   change */

	CLKPR = 1 << CLKPCE;
	/* We start with a 1 MHz/8 clock. Disable the prescaler. */
	CLKPR = 0;

	get_sernum();
}

void spi_begin(void)
{
	if (!spi_initialized)
		spi_init();
	CLR(nSS);
}

void spi_off(void)
{
	spi_initialized = 0;
	UCSR1B = 0;
}

void spi_init(void)
{
	SET(nSS);
	OUT(SCLK);
	OUT(MOSI);
	OUT(nSS);
	IN(MISO);

	UBRR1 = 0;	/* set bit rate to zero to begin */
	UCSR1C = 1 << UMSEL11 | 1 << UMSEL10;
			/* set MSPI, MSB first, SPI data mode 0 */
	UCSR1B = 1 << RXEN1 | 1 << TXEN1;
			/* enable receiver and transmitter */
	UBRR1 = 0;	/* reconfirm the bit rate */

	spi_initialized = 1;
}

void usb_init(void)
{
	USBCON |= 1 << FRZCLK;		/* freeze the clock */

	/* enable the PLL and wait for it to lock */
	PLLCSR = (1<<PLLE)|(1<<PLLP0);
	while (!(PLLCSR & (1 << PLOCK)));

	USBCON &= ~(1 << USBE);		/* reset the controller */
	USBCON |= 1 << USBE;

	USBCON &= ~(1 << FRZCLK);	/* thaw the clock */

	UDCON &= ~(1 << DETACH);	/* attach the pull-up */
	UDIEN = 1 << EORSTE;		/* enable device interrupts  */
//	UDCON |= 1 << RSTCPU;		/* reset CPU on bus reset */

	ep_init();
}

void board_app_init(void)
{
	/* enable INT0, trigger on rising edge */
	EICRA = 1 << ISC01 | 1 << ISC00;
	EIMSK = 1 << 0;
}
