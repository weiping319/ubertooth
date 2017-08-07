/*
 * Copyright 2010, 2011 Michael Ossmann
 *
 * This file is part of Project Ubertooth.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "ubertooth.h"
#include <err.h>
#include <getopt.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>


extern FILE *dumpfile;
extern FILE *infile;
extern int max_ac_errors;
static int hopCh[] = {2402, 2426, 2480};
static double scanWindow = 1000;

int devNum = 1;
struct libusb_device_handle *devh = NULL;
struct libusb_device_handle *devh1 = NULL;
struct libusb_device_handle *devh2 = NULL;

int convert_to_int (int in)
{
	int out;
	if (in >= 128)
	{
		out = (in - 256);
	}
	else
	{
		out = in;
	}
	return out;
}

static int8_t cc2400_rssi_to_dbm (const int8_t rssi)
{
	if (rssi <= -46)
		return -100;
	else if (rssi >= 34)
		return -20;
	else
		return rssi - 54;
}

int cb_legacy(usb_pkt_rx *rx)
{
	if (rx->pkt_type == BR_PACKET)
	{
		struct timeval tv;
		double now_ms;
		gettimeofday(&tv, NULL);
		now_ms = (tv.tv_sec) * 1000 + (tv.tv_usec)/1000;

		if (rx->data[38] == 0x00 && rx->data[39] == 0x3d)
		{
			int rssi = cc2400_rssi_to_dbm(convert_to_int (rx->rssi_avg));
			if (rx->data[40] < devNum)
			{
				printf ("HOP DEV: %d time: %f RSSI: %d CH: %d\n",
					rx->data[40],
					now_ms,
					rssi,
					rx->channel
					);
			}
		}
	}
}


static void usage()
{
	printf("ubertooth-rx - passive Bluetooth discovery/decode\n");
	printf("Usage:\n");
	printf("\t-h this help\n");
	printf("\t-V print version information\n");
	printf("\t-i filename\n");
	printf("\t-l <LAP> to decode (6 hex), otherwise sniff all LAPs\n");
	printf("\t-u <UAP> to decode (2 hex), otherwise try to calculate (requires LAP)\n");
	printf("\t-U <0-7> set ubertooth device to use\n");
	printf("\t-r<filename> capture packets to PCAPNG file\n");
#ifdef ENABLE_PCAP
	printf("\t-q<filename> capture packets to PCAP file\n");
#endif
	printf("\t-d<filename> dump packets to binary file\n");
	printf("\t-e max_ac_errors (default: %d, range: 0-4)\n", max_ac_errors);
	printf("\t-s reset channel scanning\n");
	printf("\t-t <SECONDS> sniff timeout - 0 means no timeout [Default: 0]\n");
	printf("\t-n number of devices\n");
	printf("\t-L legacy\n");
	printf("\t-P proposed\n");
	printf("\t-F cfo mode\n");
	printf("\t-R rssi mode\n");
	printf("\t-S sniff mode \n");
	printf("\nIf an input file is not specified, an Ubertooth device is used for live capture.\n");
}

int main(int argc, char *argv[])
{
	int opt, have_lap = 0, have_uap = 0;
	int timeout = 0;
	int reset_scan = 0;
	char *end;
	char ubertooth_device = -1;
	char ubertooth_device1 = -1;
	char ubertooth_device2 = -1;
	btbb_piconet *pn = NULL;
	uint32_t lap = 0;
	uint8_t uap = 0;
	int legacy = 0;
	int proposed = 0;
	int demo = 0;
	int cfo_mode = 0;
	int rssi_mode = 0;
	int sniff_mode = 0;
	int E = 0;

	while ((opt=getopt(argc,argv,"SFRDELPhVi:l:u:U:d:n:e:r:sq:t:")) != EOF) {
		switch(opt) {
		case 'i':
			infile = fopen(optarg, "r");
			if (infile == NULL) {
				printf("Could not open file %s\n", optarg);
				usage();
				return 1;
			}
			break;
		case 'E':
			E = 1;
			break;
		case 'l':
			lap = strtol(optarg, &end, 16);
			have_lap++;
			break;
		case 'u':
			uap = strtol(optarg, &end, 16);
			have_uap++;
			break;
		case 'U':
			if (ubertooth_device1 == -1)
				ubertooth_device1 = atoi(optarg);
			else
				ubertooth_device2 = atoi(optarg);
			break;
		case 'n':
			devNum = atoi(optarg);
			break;
		case 'r':
			if (!h_pcapng_bredr) {
				if (btbb_pcapng_create_file( optarg, "Ubertooth", &h_pcapng_bredr )) {
					err(1, "create_bredr_capture_file: ");
				}
			}
			else {
				printf("Ignoring extra capture file: %s\n", optarg);
			}
			break;
#ifdef ENABLE_PCAP
		case 'q':
			if (!h_pcap_bredr) {
				if (btbb_pcap_create_file(optarg, &h_pcap_bredr)) {
					err(1, "btbb_pcap_create_file: ");
				}
			}
			else {
				printf("Ignoring extra capture file: %s\n", optarg);
			}
			break;
#endif
		case 'd':
			dumpfile = fopen(optarg, "w");
			if (dumpfile == NULL) {
				perror(optarg);
				return 1;
			}
			break;
		case 'e':
			max_ac_errors = atoi(optarg);
			break;
		case 's':
			++reset_scan;
			break;
		case 't':
			timeout = atoi(optarg);
			break;
	
		case 'L':
			legacy = 1;
			break;
		case 'P':
			proposed = 1;
			break;
		case 'D':
			demo = 1;
 			break;
		case 'F':
			cfo_mode = 1;
			break;
		case 'R':
			rssi_mode = 1;
			break;
		case 'S':
			sniff_mode = 1;
			break;

		case 'V':
			print_version();
			return 0;
		case 'h':
		default:
			usage();
			return 1;
		}
	}

	if (have_lap) {
		pn = btbb_piconet_new();
		btbb_init_piconet(pn, lap);
		if (have_uap)
			btbb_piconet_set_uap(pn, uap);
		if (h_pcapng_bredr) {
			btbb_pcapng_record_bdaddr(h_pcapng_bredr,
						  (((uint32_t)uap)<<24)|lap,
						  have_uap ? 0xff : 0x00, 0);
		}
	} else if (have_uap) {
		printf("Error: UAP but no LAP specified\n");
		usage();
		return 1;
	}

	if (infile == NULL) 
	{
		usb_pkt_rx pkt;
		devh1 = ubertooth_start(ubertooth_device1);
		if (devh1 == NULL) 
		{
			usage();
			return 1;
		}
		if (ubertooth_device2 != -1)
		{
			devh2 = ubertooth_start(ubertooth_device2);
			if (devh2 == NULL) 
			{
				usage();
				return 1;
			}
		}
	

		/* Scan all frequencies. Same effect as
		 * ubertooth-utils -c9999. This is necessary after
		 * following a piconet. */
/*		if (reset_scan) {
			cmd_set_channel(devh, 9999);
		}
*/
		/* Clean up on exit. */

		
		if (ubertooth_device2 != -1)
		{
			register_cleanup_handler2(devh1, devh2);
		}
		else
		{
			register_cleanup_handler(devh1);
		}

		if (E == 1)
		{
			struct timeval tv;
			int hopIndex = 0;
			double now_ms, last_ms;
			cmd_rx_hop(devh1, 2402);
			gettimeofday(&tv, NULL);
			last_ms = (tv.tv_sec) * 1000 + (tv.tv_usec)/1000;

			while (1)
			{
				gettimeofday(&tv, NULL);
				now_ms = (tv.tv_sec) * 1000 + (tv.tv_usec)/1000;
				if ((now_ms - last_ms) > scanWindow)
				{
					printf("hopping\n");
					cmd_set_channel (devh1, hopCh[(++hopIndex)%3]);
					last_ms = now_ms;
				}
				int r = cmd_poll (devh1, &pkt);
				if (r == sizeof(usb_pkt_rx))
				{
					cb_legacy(&pkt);
					usleep(500);
				}
			}
			
		}
		if (proposed == 1)
		{
			if (ubertooth_device2 != -1)
			{
				rx_proposed2(devh1, devh2, pn, timeout, devNum);
		//		rx_proposed2_detection(devh1, devh2, pn, timeout);
			}
			else
				rx_proposed(devh1, pn, timeout);
		}
		else if (sniff_mode == 1)
		{ 
			rx_sniff(devh1, pn, timeout);
		}

		else if (legacy == 1)
		{ 
			rx_legacy(devh1, pn, timeout, devNum);
		}
		else if (demo == 1)
		{
			if (ubertooth_device2 != -1)
				rx_demo2(devh1, devh2, pn, timeout);
			else
				rx_demo(devh1, pn, timeout);
		}
		else if (cfo_mode == 1)
		{
			rx_cfo(devh1, pn, timeout);
		}
		else if (rssi_mode = 1)
		{
			rx_rssi(devh1, pn, timeout);
		}
		else 
		{
			rx_live(devh1, pn, timeout);
		}
//		rx_live(devh, pn, timeout);

		// Print AFH map from piconet if we have one
		if (pn)
			btbb_print_afh_map(pn);

		if (ubertooth_device1 != -1)
			ubertooth_stop(devh1);

		if (ubertooth_device2 != -1)
			ubertooth_stop(devh2);

	} 
	else 
	{
		rx_file(infile, pn);
		fclose(infile);
	}

	return 0;
}
