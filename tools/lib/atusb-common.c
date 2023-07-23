/*
 * lib/atusb-common.c - ATUSB access functions shared by all ATUSB drivers
 *
 * Written 2010-2011, 2013 by Werner Almesberger
 * Copyright 2010-2011, 2013 Werner Almesberger
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */


#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <usb.h>
#include <errno.h>

#include "atusb/ep0.h"

#include "at86rf230.h"
#include "usbopen.h"
#include "driver.h"
#include "atusb-common.h"


/* ----- error handling ---------------------------------------------------- */


int atusb_error(void *handle)
{
	struct atusb_dsc *dsc = handle;

	return dsc->error;
}


int atusb_clear_error(void *handle)
{
	struct atusb_dsc *dsc = handle;
	int ret;

	ret = dsc->error;
	dsc->error = 0;
	return ret;
}


/* ----- open/close -------------------------------------------------------- */


void *atusb_open(const char *arg)
{
	usb_dev_handle *dev;
	struct atusb_dsc *dsc;
	int res;

	usb_unrestrict();
	if (arg)
		restrict_usb_path(arg);
	dev = open_usb(ATUSB_VENDOR_ID, ATUSB_PRODUCT_ID);
	if (!dev) {
		if (errno == EPERM)
			fprintf(stderr, "Permission denied. "
			    "You may need to become root.\n");
		else
			fprintf(stderr, ":-(\n");
		return NULL;
	}

	res = usb_claim_interface(dev, 0);
	if (res == -EPERM) {
		fprintf(stderr,
		    "Permission denied. You may need to become root.\n");
		return NULL;
	}
	if (res) {
		fprintf(stderr, "usb_claim_interface: %d\n", res);
		return NULL;
	}

	dsc = malloc(sizeof(*dsc));
	if (!dsc) {
		perror("malloc");
		exit(1);
	}

	dsc->dev = dev;
	dsc->error = 0;

	atusb_driver.reg_read(dsc, REG_IRQ_STATUS);

	return dsc;
}


void atusb_close(void *handle)
{
	/* to do */
}


/* ----- device mode ------------------------------------------------------- */


void atusb_reset(void *handle)
{
	struct atusb_dsc *dsc = handle;
	int res;

	if (dsc->error)
		return;

	res =
	    usb_control_msg(dsc->dev, TO_DEV, ATUSB_RESET, 0, 0, NULL, 0, 1000);
	if (res < 0) {
		fprintf(stderr, "ATUSB_RESET: %d\n", res);
		dsc->error = 1;
	}
}


void atusb_reset_rf(void *handle)
{
	struct atusb_dsc *dsc = handle;
	int res;

	if (dsc->error)
		return;

	res = usb_control_msg(dsc->dev, TO_DEV, ATUSB_RF_RESET, 0, 0, NULL,
	    0, 1000);
	if (res < 0) {
		fprintf(stderr, "ATUSB_RF_RESET: %d\n", res);
		dsc->error = 1;
	}
}


void atusb_test_mode(void *handle)
{
	struct atusb_dsc *dsc = handle;
	int res;

	if (dsc->error)
		return;

	res =
	    usb_control_msg(dsc->dev, TO_DEV, ATUSB_TEST, 0, 0, NULL, 0, 1000);
	if (res < 0) {
		fprintf(stderr, "ATUSB_TEST: %d\n", res);
		dsc->error = 1;
	}
}


/* ----- SLP_TR ------------------------------------------------------------ */


void atusb_slp_tr(void *handle, int on, int pulse)
{
	struct atusb_dsc *dsc = handle;
	int res;

	if (dsc->error)
		return;

	if (!on || !pulse) {
		fprintf(stderr,
		    "SLP_TR mode on=%d pulse=%d not supported\n", on, pulse);
		return;
	}

	res = usb_control_msg(dsc->dev, TO_DEV, ATUSB_SLP_TR, 0, 0, NULL, 0,
	    1000);
	if (res < 0) {
		fprintf(stderr, "ATUSB_SLP_TR: %d\n", res);
		dsc->error = 1;
	}
}


/* ----- RF interrupt ------------------------------------------------------ */


/*
 * The logic here is a bit tricky. Assuming that we can get a lot of
 * interrupts, system state can change as follows:
 *
 * Event		IRQ_STATUS	EP1 on atusb	EP1 on host	irq
 *				INT					(var)
 * --------------------	------- ---	------------	-----------	-----
 * interrupt A		A	H	EP_IDLE		-		-
 * INT0 handler 	-	-	EP_TX (A)	-		-
 * interrupt B		B	H	EP_TX (A)	-		-
 * INT0 handler		B	H	EP_TX (A)	-		-
 * IN from host		B	H	EP_IDLE		A		-
 * interrupt C		B+C	H	EP_IDLE		A		-
 * call to atusb_interrupt_wait
 * read IRQ_STATUS	-	-	EP_IDLE		A		B+C
 * interrupt D		D	H	EP_IDLE		A		B+C
 * INT0 handler		-	-	EP_TX (D)	A		B+C
 * IN from host		-	-	EP_IDLE		A, D		B+C
 * usb_bulk_read	-	-	EP_IDLE		-		A+B+C+D
 * usb_bulk_read -> no more data, done
 *
 * We therefore have to consider interrupts queued up at the host and pending
 * in REG_IRQ_STATUS in addition to anything that may arrive while we wait.
 */
 

int atusb_interrupt_wait(void *handle, int timeout_ms)
{
	struct atusb_dsc *dsc = handle;
	char buf;
	int res;

	if (dsc->error)
		return 0;

	res = usb_bulk_read(dsc->dev, 1,
	    (char *) &buf, sizeof(buf), timeout_ms < 0 ? 0 : timeout_ms);
	if (res == -ETIMEDOUT)
		return 0;
	if (res < 0) {
		fprintf(stderr, "usb_bulk_read: %d\n", res);
		dsc->error = 1;
		return 0; /* handle this via atrf_error */
	}

	return atusb_driver.reg_read(handle, REG_IRQ_STATUS);
}


/* ----- CLKM handling ----------------------------------------------------- */


/*
 * ATmega32U2-based boards don't allow disabling CLKM, so we keep it at 8 MHz.
 * We could accommodate a choice between 8 MHz and 16 MHz, but that's for
 * later.
 */

int atusb_set_clkm(void *handle, int mhz)
{
	struct atusb_dsc *dsc = handle;
	uint8_t ids[3];
	int res;

	if (dsc->error)
		return 0;
	res = usb_control_msg(dsc->dev, FROM_DEV, ATUSB_ID, 0, 0,
	    (void *) ids, 3, 1000);
	if (res < 0) {
		fprintf(stderr, "ATUSB_ID: %s\n", usb_strerror());
		dsc->error = 1;
		return 0;
	}
	switch (ids[2]) {
	case ATUSB_HW_TYPE_100813:
	case ATUSB_HW_TYPE_101216:
		break;
	case ATUSB_HW_TYPE_110131:
		if (mhz == 0 || mhz == 8)
			return 1;
		fprintf(stderr, "this board only supports CLKM = 8 MHz\n");
		return 0;
	default:
		fprintf(stderr,
		    "atusb_set_clkm: unknown hardware type 0x%02x\n", ids[2]);
		return 0;
	}
	return atrf_set_clkm_generic(atusb_driver.reg_write, dsc, mhz);
}


/* ----- HardMAC ----------------------------------------------------------- */


void atusb_rx_mode(void *handle, int on)
{
	struct atusb_dsc *dsc = handle;
	int res;

	if (dsc->error)
		return;

	res = usb_control_msg(dsc->dev, TO_DEV, ATUSB_RX_MODE,
	    on, 0, NULL, 0, 1000);
	if (res < 0) {
		fprintf(stderr, "ATUSB_RX_MODE: %d\n", res);
		dsc->error = 1;
	}
}


int atusb_rx(void *handle, void *buf, int size, int timeout_ms, uint8_t *lqi)
{
	struct atusb_dsc *dsc = handle;
	uint8_t len;
	int res;
	uint8_t tmp[MAX_PSDU+2]; /* PHR, LQI */

	/*
	 * Seems that either the USB stack or libusb doesn't like it if we do a
	 * read of size one followed by the full read. Therefore, we just do
	 * a maximum-sized read and hope that we don't split packets.
	 */
	res = usb_bulk_read(dsc->dev, 1, (char *) tmp, sizeof(tmp), timeout_ms);
	if (res == -ETIMEDOUT)
		return 0;
	if (res < 0) {
		fprintf(stderr, "usb_bulk_read: %d\n", res);
		dsc->error = 1;
		return 0;
	}

	len = tmp[0];
	if (len & 0x80) {
		fprintf(stderr, "atusb_rx: invalid length 0x%02x\n", len);
		return 0;
	}
	if (len > size) {
		fprintf(stderr, "atusb_rx: len %u > size %d\n", len, size);
		return 0;
	}
	if (len > res+2) {
		fprintf(stderr, "atusb_rx: len %u > res %d+2\n", len, res);
		return 0;
	}

	memcpy(buf, tmp+1, len);
	if (lqi)
		*lqi = tmp[len+1];

	return len;
}


void atusb_tx(void *handle, const void *buf, int size)
{
	struct atusb_dsc *dsc = handle;
	uint8_t tmp;
	int res;

	if (dsc->error)
		return;

	res = usb_control_msg(dsc->dev, TO_DEV, ATUSB_TX,
	    0, 0, (void *) buf, size, 1000);
	if (res < 0) {
		fprintf(stderr, "ATUSB_TX: %d\n", res);
		dsc->error = 1;
	}
	res = usb_bulk_read(dsc->dev, 1, (char *) &tmp, 1, 0);
	if (res < 0) {
		fprintf(stderr, "usb_bulk_read: %d\n", res);
		dsc->error = 1;
		return;
	}
	if (tmp)
		fprintf(stderr, "atusb_tx: ACK is non-zero 0x%02x\n", tmp);
}


/* ----- Driver-specific hacks --------------------------------------------- */


void *atusb_dev_handle(void *handle)
{
	struct atusb_dsc *dsc = handle;

	return dsc->dev;
}
