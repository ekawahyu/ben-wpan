/*
 * atrf-xtal/atrf-xtal.c - AT86RF230/1 crystal diagnostic utility
 *
 * Written 2011 by Werner Almesberger
 * Copyright 2011 Werner Almesberger
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */


#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "atrf.h"

#include "atrf-xtal.h"


#define	DEFAULT_SIZE	127
#define	DEFAULT_TRIM	8


static void usage(const char *name)
{
	fprintf(stderr,
"usage: %s [-b count [-p ppm]] [-d driver[:arg]] [-r] [-s size] [-t trim]\n"
"       %*s [repetitions]\n\n"
"  -b count         base count for relative result\n"
"  -d driver[:arg]  use the specified driver (default: %s)\n"
"  -p ppm           maximum deviation from base count\n"
"  -r               instead of printing a mean value, dump the raw samples\n"
"  -s size          payload size in bytes, 0-127 (default: %d bytes)\n"
"  -t trim          trim capacitor setting, 0-15 (default: %d)\n"
"  repetitions      number of measurements (default: 1)\n"
    , name, (int) strlen(name), "",
    atrf_default_driver_name(), DEFAULT_SIZE, DEFAULT_TRIM);
	exit(1);
}


int main(int argc, char *const *argv)
{
	const char *driver = NULL;
	struct atrf_dsc *dsc;
	int size = DEFAULT_SIZE;
	int trim = DEFAULT_TRIM;
	double base = 0, ppm = 0;
	int rep = 1;
	int dump_raw = 0;
	char *end;
	int c;

	while ((c = getopt(argc, argv, "b:d:p:rs:t:")) != EOF)
		switch (c) {
		case 'b':
			base = strtof(optarg, &end);
			if (*end)
				usage(*argv);
			break;
		case 'd':
			driver = optarg;
			break;
		case 'p':
			ppm = strtof(optarg, &end);
			if (*end)
				usage(*argv);
			break;
		case 'r':
			dump_raw = 1;
			break;
		case 's':
			size = strtoul(optarg, &end, 0);
			if (*end)
				usage(*argv);
			if (size > 127)
				usage(*argv);
			break;
		case 't':
			trim = strtoul(optarg, &end, 0);
			if (*end)
				usage(*argv);
			if (trim > 15)
				usage(*argv);
			break;
		default:
			usage(*argv);
		}

	if (ppm && !base)
		usage(*argv);

	switch (argc-optind) {
	case 0:
		break;
	case 1:
		rep = strtoul(argv[optind], &end, 0);
		if (*end)
			usage(*argv);
		break;
	default:
		usage(*argv);
	}

	dsc = atrf_open(driver);
	if (!dsc)
		return 1;

	do_atben(dsc, size, trim, rep, dump_raw, base, ppm);

	return 0;
}
