/*
 * fw/mac.c - HardMAC functions
 *
 * Written 2011, 2013 by Werner Almesberger
 * Copyright 2011, 2013 Werner Almesberger
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "usb.h"

#include "at86rf230.h"
#include "spi.h"
#include "board.h"
#include "mac.h"


bool (*mac_irq)(void) = NULL;


static uint8_t rx_buf[MAX_PSDU+2]; /* PHDR+payload+LQ */
static uint8_t tx_buf[MAX_PSDU];
static uint8_t tx_size = 0;
static bool txing = 0;
static bool queued_tx_ack = 0;


static uint8_t reg_read(uint8_t reg)
{
	uint8_t value;

	spi_begin();
	spi_send(AT86RF230_REG_READ | reg);
	value = spi_recv();
	spi_end();

	return value;
}


static void reg_write(uint8_t reg, uint8_t value)
{
	spi_begin();
	spi_send(AT86RF230_REG_WRITE | reg);
	spi_send(value);
	spi_end();
}


static void rx_done(void *user)
{
	led(0);
	if (queued_tx_ack) {
		usb_send(&eps[1], "", 1, rx_done, NULL);
		queued_tx_ack = 0;	
	}
}


static void receive_frame(void)
{
	uint8_t size, i;

	spi_begin();
	if (!(spi_io(AT86RF230_BUF_READ) & RX_CRC_VALID)) {
		spi_end();
		return;
	}
	size = spi_recv();
	if (!size || (size & 0x80)) {
		spi_end();
		return;
	}

	rx_buf[0] = size;
	for (i = 0; i != size+1; i++)
		rx_buf[i+1] = spi_recv();
	spi_end();
	led(1);
	usb_send(&eps[1], rx_buf, size+2, rx_done, NULL);
}


static bool handle_irq(void)
{
	uint8_t irq;

	irq = reg_read(REG_IRQ_STATUS);
	if (!(irq & IRQ_TRX_END))
		return 1;

	if (txing) {
		if (eps[1].state == EP_IDLE)
			usb_send(&eps[1], "", 1, rx_done, NULL);
		else {
			if (queued_tx_ack)
				panic();
			queued_tx_ack = 1;
		}
		txing = 0;
		return 1;
	}

	/* unlikely */
	if (eps[1].state != EP_IDLE)
		return 1;

	receive_frame();

	return 1;
}


static void change_state(uint8_t new)
{
	while ((reg_read(REG_TRX_STATUS) & TRX_STATUS_MASK) ==
	    TRX_STATUS_TRANSITION);
	reg_write(REG_TRX_STATE, new);
}


bool mac_rx(int on)
{
	if (on) {
		mac_irq = handle_irq;
		reg_read(REG_IRQ_STATUS);
		change_state(TRX_CMD_RX_ON);
	} else {
		mac_irq = NULL;
		change_state(TRX_CMD_FORCE_TRX_OFF);
		txing = 0;
	}
	return 1;
}


static void do_tx(void *user)
{
	uint16_t timeout = 0xffff;
	uint8_t status;
	uint8_t i;

	/*
	 * If we time out here, the host driver will time out waiting for the
	 * TRX_END acknowledgement.
	 */
	do {
		if (!--timeout)
			return;
		status = reg_read(REG_TRX_STATUS) & TRX_STATUS_MASK;
	}
	while (status != TRX_STATUS_RX_ON && status != TRX_STATUS_RX_AACK_ON);

	/*
	 * We use TRX_CMD_FORCE_PLL_ON instead of TRX_CMD_PLL_ON because a new
	 * reception may have begun while we were still working on the previous
	 * one.
	 */
	reg_write(REG_TRX_STATE, TRX_CMD_FORCE_PLL_ON);

	handle_irq();

	spi_begin();
	spi_send(AT86RF230_BUF_WRITE);
	spi_send(tx_size+2); /* CRC */
	for (i = 0; i != tx_size; i++)
		spi_send(tx_buf[i]);
	spi_end();

	slp_tr();

	txing = 1;

	/*
	 * Wait until we reach BUSY_TX, so that we command the transition to
	 * RX_ON which will be executed upon TX completion.
	 */
	change_state(TRX_CMD_RX_ON);
}


bool mac_tx(uint16_t flags, uint16_t len)
{
	if (len > MAX_PSDU)
		return 0;
	tx_size = len;
	usb_recv(&eps[0], tx_buf, len, do_tx, NULL);
	return 1;
}


void mac_reset(void)
{
	mac_irq = NULL;
	txing = 0;
	queued_tx_ack = 0;

	/* enable CRC and PHY_RSSI (with RX_CRC_VALID) in SPI status return */
	reg_write(REG_TRX_CTRL_1,
	    TX_AUTO_CRC_ON | SPI_CMD_MODE_PHY_RSSI << SPI_CMD_MODE_SHIFT);
}
