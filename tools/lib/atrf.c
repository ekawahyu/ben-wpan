/*
 * lib/atrf.c - ATRF access functions library
 *
 * Written 2010-2011, 2013 by Werner Almesberger
 * Copyright 2010-2011, 2013 Werner Almesberger
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */


#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "at86rf230.h"

#include "driver.h"
#include "atrf.h"


struct atrf_dsc {
	const struct atrf_driver *driver;
	void *handle;
	char *spec;
	enum atrf_chip_id chip_id;
};


static const struct atrf_driver *drivers[] = {
#ifdef HAVE_BEN
	&atben_driver,
#endif
#ifdef HAVE_USB
	&atusb_driver,
	&atusb_spi_driver,
#endif
	&atnet_driver,
	NULL
};


void *atrf_usb_handle(struct atrf_dsc *dsc)
{
#ifdef HAVE_USB
	if (dsc->driver == &atusb_driver || dsc->driver == &atusb_spi_driver)
		return atusb_dev_handle(dsc->handle);
#endif
	return NULL;
}


void *atrf_ben_regs(struct atrf_dsc *dsc)
{
#ifdef HAVE_BEN
	if (dsc->driver == &atben_driver)
		return atben_regs(dsc->handle);
#endif
	return NULL;
}


int atrf_error(struct atrf_dsc *dsc)
{
	return dsc->driver->error ? dsc->driver->error(dsc->handle) : 0;
}


int atrf_clear_error(struct atrf_dsc *dsc)
{
	return dsc->driver->clear_error ?
	    dsc->driver->clear_error(dsc->handle) : 0;
}


static enum atrf_chip_id identify(struct atrf_dsc *dsc)
{
	uint8_t part, version;

	part = atrf_reg_read(dsc, REG_PART_NUM);
	version = atrf_reg_read(dsc, REG_VERSION_NUM);
	switch (part) {
	case 2:	/* AT86RF230 */
		switch (version) {
		case 1: /* rev A */
		case 2: /* rev B */
			return atrf_at86rf230;
		default:
			return atrf_unknown_chip;
		}
		break;
	case 3:	/* AT86RF231 */
		switch (version) {
		case 2: /* rev A */
			return atrf_at86rf231;
		default:
			return atrf_unknown_chip;
		}
		break;
	case 7:	/* AT86RF212 */
		switch (version) {
		case 1: /* rev A */
		case 3: /* rev C */
			return atrf_at86rf212;
		default:
			return atrf_unknown_chip;
		}
		break;
	default:
		return atrf_unknown_chip;
	}
	return atrf_unknown_chip;
}


const char *atrf_default_driver_name(void)
{
	return drivers[0] ? drivers[0]->name : "none";
}


static const struct atrf_driver *select_driver(const char *spec,
    const char **opt)
{
	const struct atrf_driver **drv;
	const char *end;
	size_t len;

	if (!*drivers) {
		fprintf(stderr, "no drivers defined\n");
		return NULL;
	}

	*opt = NULL;
	if (!spec || !strcmp(spec, "default"))
		return *drivers;
	
	end = strchr(spec, ':');
	if (!end)
		end = strchr(spec, 0);
	len = end-spec;
	for (drv = drivers; *drv; drv++)
		if (!strncmp((*drv)->name, spec, len) &&
		    strlen((*drv)->name) == len)
			break;
	if (!*drv) {
		fprintf(stderr, "no driver \"%.*s\" found\n", (int) len, spec);
		return NULL;
	}
	if (*end)
		*opt = end+1;
	return *drv;
}


struct atrf_dsc *atrf_open(const char *spec)
{
	struct atrf_dsc *dsc;
	const struct atrf_driver *driver;
	const char *opt;
	void *handle;

	driver = select_driver(spec, &opt);
	if (!driver)
		return NULL;
	handle = driver->open(opt);
	if (!handle)
		return NULL;
	dsc = malloc(sizeof(*dsc));
	if (!dsc) {
		perror("malloc");
		exit(1);
	}
	dsc->driver = driver;
	dsc->handle = handle;
	if (spec) {
		dsc->spec = strdup(spec);
		if (!dsc->spec) {
			perror("strdup");
			exit(1);
		}
	} else {
		dsc->spec= NULL;
	}
	dsc->chip_id = identify(dsc);
	return dsc;
}


void atrf_close(struct atrf_dsc *dsc)
{
	if (dsc->driver->close)
		dsc->driver->close(dsc->handle);
	free(dsc->spec);
	free(dsc);
}


const char *atrf_driver_spec(struct atrf_dsc *dsc, int last)
{
	if (!dsc->spec)
		return dsc->driver->name;
	if (!last || !dsc->driver->driver_spec)
		return dsc->spec;
	return dsc->driver->driver_spec(dsc->handle);
}


void atrf_reset(struct atrf_dsc *dsc)
{
	if (dsc->driver->reset)
		dsc->driver->reset(dsc->handle);
}


void atrf_reset_rf(struct atrf_dsc *dsc)
{
	dsc->driver->reset_rf(dsc->handle);
}


enum atrf_chip_id atrf_identify(struct atrf_dsc *dsc)
{
	return dsc->chip_id;
}


int atrf_test_mode(struct atrf_dsc *dsc)
{
	if (!dsc->driver->test_mode)
		return 0;
	dsc->driver->test_mode(dsc->handle);
	return 1;
}


int atrf_slp_tr(struct atrf_dsc *dsc, int on, int pulse)
{
	if (!dsc->driver->slp_tr)
		return 0;
	dsc->driver->slp_tr(dsc->handle, on, pulse);
	return 1;
}


int atrf_set_clkm_generic(
    void (*reg_write)(void *dsc, uint8_t reg, uint8_t value),
    void *handle, int mhz)
{
	uint8_t clkm;

	if (!mhz) {
		reg_write(handle, REG_TRX_CTRL_0, 0); /* disable CLKM */
		return 1;
	}
	switch (mhz) {
	case 1:
		clkm = CLKM_CTRL_1MHz;
		break;
	case 2:
		clkm = CLKM_CTRL_2MHz;
		break;
	case 4:
		clkm = CLKM_CTRL_4MHz;
		break;
	case 8:
		clkm = CLKM_CTRL_8MHz;
		break;
	case 16:
		clkm = CLKM_CTRL_16MHz;
		break;
	default:
		fprintf(stderr, "unsupported CLKM frequency %d MHz\n", mhz);
		return 0;
	}
	reg_write(handle, REG_TRX_CTRL_0,
	    (PAD_IO_8mA << PAD_IO_CLKM_SHIFT) | clkm);
	return 1;
}


int atrf_set_clkm(struct atrf_dsc *dsc, int mhz)
{
	if (dsc->driver->set_clkm)
		return dsc->driver->set_clkm(dsc->handle, mhz);
	else
		return atrf_set_clkm_generic(dsc->driver->reg_write,
		    dsc->handle, mhz);
}


void atrf_reg_write(struct atrf_dsc *dsc, uint8_t reg, uint8_t value)
{
	dsc->driver->reg_write(dsc->handle, reg, value);
}


uint8_t atrf_reg_read(struct atrf_dsc *dsc, uint8_t reg)
{
	return dsc->driver->reg_read(dsc->handle, reg);
}


void atrf_buf_write(struct atrf_dsc *dsc, const void *buf, int size)
{
	dsc->driver->buf_write(dsc->handle, buf, size);
}


int atrf_buf_read(struct atrf_dsc *dsc, void *buf, int size)
{
	return dsc->driver->buf_read(dsc->handle, buf, size);
}


void atrf_sram_write(struct atrf_dsc *dsc, uint8_t addr, uint8_t value)
{
	dsc->driver->sram_write(dsc->handle, addr, value);
}


uint8_t atrf_sram_read(struct atrf_dsc *dsc, uint8_t addr)
{
	return dsc->driver->sram_read(dsc->handle, addr);
}


int atrf_interrupt_wait(struct atrf_dsc *dsc, int timeout_ms)
{
	return dsc->driver->interrupt_wait(dsc->handle, timeout_ms);
}


void atrf_rx_mode(struct atrf_dsc *dsc, int on)
{
	if (dsc->driver->rx_mode)
		dsc->driver->rx_mode(dsc->handle, on);
}


int atrf_rx(struct atrf_dsc *dsc, void *buf, int size, int timeout_ms,
    uint8_t *lqi)
{
	if (!dsc->driver->rx)
		return 0;
	return dsc->driver->rx(dsc->handle, buf, size, timeout_ms, lqi);
}


void atrf_tx(struct atrf_dsc *dsc, const void *buf, int size)
{
	if (dsc->driver->tx)
		dsc->driver->tx(dsc->handle, buf, size);
}
