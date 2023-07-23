/*
 * lib/atnet.c - Access functions library for network proxy
 *
 * Written 2011 by Werner Almesberger
 * Copyright 2011 Werner Almesberger
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */


#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>

#include "netio.h"

#include "driver.h"


#define	DEFAULT_PORT	"5440"


struct atnet_dsc {
	struct netio *netio;
	int error;
	char *spec;
	char reply[1000];
};


static int dialog_vsend(struct atnet_dsc *dsc, const char *fmt, va_list ap)
{
	va_list ap2;
	char *buf;
	int n, res;

	va_copy(ap2, ap);
	n = vsnprintf(NULL, 0, fmt, ap2);

	buf = malloc(n+1);
	if (!buf) {
		perror("malloc");
		return -1;
	}

	vsprintf(buf, fmt, ap);

	buf[n] = '\n';
	res = netio_write(dsc->netio, buf, n+1);
	free(buf);

	return res < 0 ? -1 : 0;
}


static int dialog_send(struct atnet_dsc *dsc, const char *fmt, ...)
{
	va_list ap;
	int res;

	va_start(ap, fmt);
	res = dialog_vsend(dsc, fmt, ap);
	va_end(ap);
	return res;
}


static int dialog_recv(struct atnet_dsc *dsc)
{
	int n;

	n = netio_read_until(dsc->netio,
	    "\n", dsc->reply, sizeof(dsc->reply)-1, NULL);
	if (n < 0)
		return -1;
	dsc->reply[n] = 0;

	return dsc->reply[0] == '+' ? 0 : -1;
}


static int dialog(struct atnet_dsc *dsc, const char *fmt, ...)
{
	va_list ap;
	int res;

	va_start(ap, fmt);
	res = dialog_vsend(dsc, fmt, ap);
	va_end(ap);
	if (res < 0)
		return res;

	return dialog_recv(dsc);
}


/* ----- error handling ---------------------------------------------------- */


static int atnet_error(void *handle)
{
	struct atnet_dsc *dsc = handle;

	return dsc->error;
}


static int atnet_clear_error(void *handle)
{
	struct atnet_dsc *dsc = handle;
	int ret;

	ret = dsc->error;
	dsc->error = 0;
	return ret;
}


/* ----- open/close -------------------------------------------------------- */


static void *atnet_open(const char *arg)
{
	char *host = NULL, *comma;
	const char *port = DEFAULT_PORT;
	struct addrinfo *addrs;
	const struct addrinfo *addr;
	static struct addrinfo hint = { .ai_socktype = SOCK_STREAM };
	int res, s;
	struct atnet_dsc *dsc;

	if (arg) {
		host = strdup(arg);
		if (!host) {
			perror("strdup");
			return NULL;
		}
		comma = strchr(host, ',');
		if (comma) {
			*comma = 0;
			port = comma+1;
		}
	}
	res = getaddrinfo(arg && *host ? host : NULL, port, &hint, &addrs);
	if (res < 0) {
		fprintf(stderr, "%s: %s\n", arg, gai_strerror(res));
		free(host);
		return NULL;
	}
	free(host);

	for (addr = addrs; addr; addr = addr->ai_next) {
		s = socket(addr->ai_family, addr->ai_socktype,
		    addr->ai_protocol);
		if (s < 0)
			continue;
		if (connect(s, addr->ai_addr, addr->ai_addrlen) >= 0)
			break;
		close(s);
	}
	if (!addr) {
		perror(arg);
		freeaddrinfo(addrs);
		return NULL;
	}

	freeaddrinfo(addrs);

	dsc = malloc(sizeof(*dsc));
	if (!dsc) {
		perror("malloc");
		exit(1);
	}

	dsc->netio = netio_open(s);
	if (!dsc->netio) {
		free(dsc);
		return NULL;
	}

	if (dialog_recv(dsc) < 0) {
		netio_close(dsc->netio);
		free(dsc);
		return NULL;
	}

	dsc->error = 0;
	dsc->spec = NULL;
	*dsc->reply = 0;

	return dsc;
}


static void atnet_close(void *handle)
{
	struct atnet_dsc *dsc = handle;

	netio_close(dsc->netio);
	free(dsc->spec);
}


/* ----- driver specification ---------------------------------------------- */


static const char *atnet_driver_spec(void *handle)
{
	struct atnet_dsc *dsc = handle;

	if (dialog(dsc, "SPEC") < 0) {
		dsc->error = 1;
		return NULL;
	}
	free(dsc->spec);
	dsc->spec = strdup(dsc->reply+1);
	if (!dsc->spec) {
		perror("strdup");
		exit(1);
	}
	return dsc->spec;
}


/* ----- device mode ------------------------------------------------------- */


static void atnet_reset(void *handle)
{
	struct atnet_dsc *dsc = handle;

	if (dsc->error)
		return;
	if (dialog(dsc, "RESET") < 0)
		dsc->error = 1;
}


static void atnet_reset_rf(void *handle)
{
	struct atnet_dsc *dsc = handle;

	if (dsc->error)
		return;
	if (dialog(dsc, "RESET_RF") < 0)
		dsc->error = 1;
}


static void atnet_test_mode(void *handle)
{
	struct atnet_dsc *dsc = handle;

	if (dsc->error)
		return;
	if (dialog(dsc, "TEST") < 0)
		dsc->error = 1;
}


static void atnet_slp_tr(void *handle, int on, int pulse)
{
	struct atnet_dsc *dsc = handle;

	if (dsc->error)
		return;
	if (dialog(dsc, "SLP_TR %d %d", on, pulse) < 0)
		dsc->error = 1;
}


/* ----- register access --------------------------------------------------- */


static void atnet_reg_write(void *handle, uint8_t reg, uint8_t value)
{
	struct atnet_dsc *dsc = handle;

	if (dsc->error)
		return;
	if (dialog(dsc, "SET 0x%02x 0x%02x", reg, value) < 0)
		dsc->error = 1;
}


static uint8_t atnet_reg_read(void *handle, uint8_t reg)
{
	struct atnet_dsc *dsc = handle;
	unsigned long value;
	char *end;

	if (dsc->error)
		return 0;
	if (dialog(dsc, "GET 0x%02x", reg) < 0) {
		dsc->error = 1;
		return 0;
	}
	value = strtoul(dsc->reply+1, &end, 0);
	if (*end || value > 255) {
		fprintf(stderr, "invalid response \"%s\"\n", dsc->reply+1);
		dsc->error = 1;
		return 0;
	}
	return value;
}


/* ----- frame buffer access ----------------------------------------------- */


static void atnet_buf_write(void *handle, const void *buf, int size)
{
	struct atnet_dsc *dsc = handle;
	char tmp[20];
	int n;

	if (dsc->error)
		return;

	n = snprintf(tmp, sizeof(tmp), "WRITE %d ", size);
	assert(n < sizeof(tmp));
	if (netio_write(dsc->netio, tmp, n) < 0) {
		dsc->error = 1;
                return;
	}

	if (netio_write(dsc->netio, buf, size) < 0) {
		dsc->error = 1;
                return;
	}

	if (netio_write(dsc->netio, "\n", 1) < 0) {
		dsc->error = 1;
                return;
	}

	if (dialog_recv(dsc) < 0)
		dsc->error = 1;
}


static int atnet_buf_read(void *handle, void *buf, int size)
{
	struct atnet_dsc *dsc = handle;
	uint8_t tmp[200];
	int n, got = 0;
	unsigned long len = 0;
	char *end;

	if (dsc->error)
		return -1;

	if (dialog_send(dsc, "READ") < 0)
		goto fail;

	n = netio_read_until(dsc->netio, " ", tmp, sizeof(tmp)-1, NULL);
	if (n < 0)
		goto fail;
	tmp[n] = 0;
	if (*tmp == '-') {
		fprintf(stderr, "%s ", tmp+1);
		n = netio_read_until(dsc->netio,
		    "\n", tmp, sizeof(tmp)-1, NULL);
		if (n >= 0) {
			tmp[n] = 0;
			fprintf(stderr, "%s\n", tmp);
		}
		goto fail;
	}
	if (*tmp != '+' || n < 2) /* +0<spc> */
		goto invalid;
	len = strtoul(tmp+1, &end, 0);
	if (*end)
		goto invalid;
	if (len > size) {
		fprintf(stderr, "buffer overflow\n");
		goto fail;
	}

	got = netio_read(dsc->netio, buf, len);
	if (got < 0)
		goto fail;

	if (netio_getc(dsc->netio, tmp) < 0)
		goto fail;
	if (*tmp == '\n')
		return len;

invalid:
	fprintf(stderr, "invalid reponse\n");
fail:
	dsc->error = 1;
	return -1;
}


/* ----- SRAM access ------------------------------------------------------- */


static void atnet_sram_write(void *handle, uint8_t addr, uint8_t value)
{
	struct atnet_dsc *dsc = handle;

	if (dsc->error)
		return;
	if (dialog(dsc, "SETRAM 0x%02x 0x%02x", addr, value) < 0)
		dsc->error = 1;
}


static uint8_t atnet_sram_read(void *handle, uint8_t addr)
{
	struct atnet_dsc *dsc = handle;
	unsigned long value;
	char *end;

	if (dsc->error)
		return 0;
	if (dialog(dsc, "GETRAM 0x%02x", addr) < 0) {
		dsc->error = 1;
		return 0;
	}
	value = strtoul(dsc->reply+1, &end, 0);
	if (*end || value > 255) {
		fprintf(stderr, "invalid response \"%s\"\n", dsc->reply+1);
		dsc->error = 1;
		return 0;
	}
	return value;
}


/* ----- RF interrupt ------------------------------------------------------ */


int atnet_interrupt_wait(void *handle, int timeout_ms)
{
	struct atnet_dsc *dsc = handle;
	unsigned long value;
	char *end;

	if (dsc->error)
		return 0;
	if (dialog(dsc, "WAIT %d", timeout_ms) < 0)
		return 0;
	value = strtoul(dsc->reply+1, &end, 0);
	if (*end || value > 0xff) {
		fprintf(stderr, "invalid response \"%s\"\n", dsc->reply+1);
		dsc->error = 1;
		return 0;
	}
	return value;
}


/* ----- CLKM handling ----------------------------------------------------- */


static int atnet_set_clkm(void *handle, int mhz)
{
	struct atnet_dsc *dsc = handle;

	return dialog(dsc, "CLKM %d", mhz) == 0;
}


/* ----- driver interface -------------------------------------------------- */


struct atrf_driver atnet_driver = {
	.name		= "net",
	.open		= atnet_open,
	.close		= atnet_close,
	.driver_spec	= atnet_driver_spec,
	.error		= atnet_error,
	.clear_error	= atnet_clear_error,
	.reset		= atnet_reset,
	.reset_rf	= atnet_reset_rf,
	.test_mode	= atnet_test_mode,
	.slp_tr		= atnet_slp_tr,
	.set_clkm	= atnet_set_clkm,
	.reg_write	= atnet_reg_write,
	.reg_read	= atnet_reg_read,
	.buf_write	= atnet_buf_write,
	.buf_read	= atnet_buf_read,
	.sram_write	= atnet_sram_write,
	.sram_read	= atnet_sram_read,
	.interrupt_wait	= atnet_interrupt_wait,
};
