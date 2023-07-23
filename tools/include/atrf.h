/*
 * include/atrf.h - ATRF access functions library
 *
 * Written 2010-2011, 2013 by Werner Almesberger
 * Copyright 2010-2011, 2013 Werner Almesberger
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef ATRF_H
#define	ATRF_H

#include <stdint.h>


enum atrf_chip_id {
	atrf_unknown_chip	= 0,
	atrf_at86rf230		= 1,
	atrf_at86rf231		= 2,
	atrf_at86rf212		= 7,
};


struct atrf_dsc;


void *atrf_usb_handle(struct atrf_dsc *dsc); /* hack for atrf-id */
void *atrf_ben_regs(struct atrf_dsc *dsc); /* hack for atrf-xtal */

const char *atrf_default_driver_name(void);
struct atrf_dsc *atrf_open(const char *spec);
void atrf_close(struct atrf_dsc *dsc);
const char *atrf_driver_spec(struct atrf_dsc *dsc, int last);

int atrf_error(struct atrf_dsc *dsc);
int atrf_clear_error(struct atrf_dsc *dsc);

void atrf_reset(struct atrf_dsc *dsc);
void atrf_reset_rf(struct atrf_dsc *dsc);

enum atrf_chip_id atrf_identify(struct atrf_dsc *dsc);

int atrf_test_mode(struct atrf_dsc *dsc);
int atrf_slp_tr(struct atrf_dsc *dsc, int on, int pulse);
int atrf_set_clkm(struct atrf_dsc *dsc, int mhz);

void atrf_reg_write(struct atrf_dsc *dsc, uint8_t reg, uint8_t value);
uint8_t atrf_reg_read(struct atrf_dsc *dsc, uint8_t reg);

void atrf_buf_write(struct atrf_dsc *dsc, const void *buf, int size);
int atrf_buf_read(struct atrf_dsc *dsc, void *buf, int size);

void atrf_sram_write(struct atrf_dsc *dsc, uint8_t addr, uint8_t value);
uint8_t atrf_sram_read(struct atrf_dsc *dsc, uint8_t addr);

int atrf_interrupt_wait(struct atrf_dsc *dsc, int timeout_ms);

/* HardMAC operations */

void atrf_rx_mode(struct atrf_dsc *dsc, int on);
int atrf_rx(struct atrf_dsc *dsc, void *buf, int size, int timeout_ms,
    uint8_t *lqi);
void atrf_tx(struct atrf_dsc *dsc, const void *buf, int size);

#endif /* !ATRF_H */
