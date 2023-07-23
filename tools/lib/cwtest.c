/*
 * lib/cwtest.c - Set up AT86RF230/231 constant wave test mode
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
#include <unistd.h>

#include "at86rf230.h"
#include "atrf.h"
#include "misctxrx.h"

#include "cwtest.h"


static int last_cont_tx; /* @@@ hack for resuming on the 230 */


int cw_test_needs_reset(struct atrf_dsc *dsc)
{
	if (atrf_usb_handle(dsc))
		return 1;
	if (atrf_identify(dsc) == atrf_at86rf230)
		return 1;
	return 0;
}


static void enter_test_mode_230(struct atrf_dsc *dsc, uint8_t cont_tx)
{
	atrf_buf_write(dsc, "", 1);
	atrf_reg_write(dsc, REG_CONT_TX_0, CONT_TX_MAGIC);
	atrf_reg_write(dsc, REG_CONT_TX_1, cont_tx);

	if (!atrf_test_mode(dsc)) {
		atrf_reset_rf(dsc);
		fprintf(stderr, "device does not support test mode\n");
		exit(1);
	}

	atrf_reg_write(dsc, REG_TRX_STATE, TRX_CMD_PLL_ON);
	wait_for_interrupt(dsc, IRQ_PLL_LOCK, IRQ_PLL_LOCK, 0);

	atrf_reg_write(dsc, REG_TRX_STATE, TRX_CMD_TX_START);
}


static void prepare_test_mode_231(struct atrf_dsc *dsc, uint8_t cont_tx)
{
	uint8_t buf[127];
	uint8_t status;

	switch (cont_tx) {
	case CONT_TX_M2M:
		fprintf(stderr,
		    "-2 MHz mode is not supported by the AT86RF231\n");
		atrf_close(dsc);
		exit(1);
	case CONT_TX_M500K:
		memset(buf, 0, sizeof(buf));
		break;
	case CONT_TX_P500K:
		memset(buf, 0xff, sizeof(buf));
		break;
	default:
		abort();
	}

	atrf_reg_write(dsc, REG_IRQ_MASK, IRQ_PLL_LOCK);		/* 2 */
	atrf_reg_write(dsc, REG_TRX_CTRL_1, 0);				/* 3 */
	atrf_reg_write(dsc, REG_TRX_STATE, TRX_CMD_FORCE_TRX_OFF);	/* 4 */
	/* deleted step 5 - we don't need to enable CLKM */

	status = atrf_reg_read(dsc, REG_TRX_STATUS) & TRX_STATUS_MASK;	/* 8 */
	if (status != TRX_STATUS_TRX_OFF) {
		fprintf(stderr, "expected status 0x%02x, got 0x%02x\n",
		    TRX_STATUS_TRX_OFF, status);
		exit(1);
	}

	atrf_reg_write(dsc, REG_CONT_TX_0, CONT_TX_MAGIC);		/* 9 */
	atrf_reg_write(dsc, REG_TRX_CTRL_2, OQPSK_DATA_RATE_2000);	/*10 */
	atrf_reg_write(dsc, REG_RX_CTRL, 0xa7);				/*11 */

	atrf_buf_write(dsc, buf, sizeof(buf));				/*12 */
}


static void start_test_mode_231(struct atrf_dsc *dsc)
{
	atrf_reg_write(dsc, REG_PART_NUM, 0x54);			/*13 */
	atrf_reg_write(dsc, REG_PART_NUM, 0x46);			/*14 */

	atrf_reg_write(dsc, REG_TRX_STATE, TRX_CMD_PLL_ON);		/*15 */
	wait_for_interrupt(dsc, IRQ_PLL_LOCK, IRQ_PLL_LOCK, 0);		/*16 */

	atrf_reg_write(dsc, REG_TRX_STATE, TRX_CMD_TX_START);		/*17 */
}


void cw_test_begin(struct atrf_dsc *dsc, uint8_t cont_tx)
{
	switch (atrf_identify(dsc)) {
	case atrf_at86rf230:
		enter_test_mode_230(dsc, cont_tx);
		last_cont_tx = cont_tx;
		break;
	case atrf_at86rf231:
		prepare_test_mode_231(dsc, cont_tx);
		start_test_mode_231(dsc);
		break;
	default:
		abort();
	}
}


void cw_test_resume(struct atrf_dsc *dsc)
{
	switch (atrf_identify(dsc)) {
	case atrf_at86rf230:
		enter_test_mode_230(dsc, last_cont_tx);
		break;
	case atrf_at86rf231:
		start_test_mode_231(dsc);
		break;
	default:
		abort();
	}
}


void cw_test_end(struct atrf_dsc *dsc)
{
	if (atrf_identify(dsc) == atrf_at86rf231)
		atrf_reg_write(dsc, REG_PART_NUM, 0);
	
	atrf_reg_write(dsc, REG_TRX_STATE, TRX_CMD_FORCE_TRX_OFF);

	/*
	 * atrf_reset_rf can take a long time. I appears that at least the
	 * AT86RF231 also exits test mode if we send it to sleep for a
	 * moment.
	 */
	if (cw_test_needs_reset(dsc))
		atrf_reset_rf(dsc);
	else {
		usleep(2);	/* table 7-1: tTR12(typ) = 1 us */
		atrf_slp_tr(dsc, 1, 0);
		usleep(10);	/* table 7-1: tTR3(typ) doesn't really apply */
		atrf_slp_tr(dsc, 0, 0);
		usleep(500);	/* table 7-1: tTR2(typ) = 380 */
	}
}
