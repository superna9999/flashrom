/*
 * This file is part of the flashrom project.
 *
 * Copyright 2021, BayLibre, SAS. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 * Driver for programming SPI flash chips attached to the Genesys Logic, Inc USB Memory
 * Card Reader Controller devices.
 *
 * Product page: http://www.genesyslogic.com/en/product_list.php?1st=1&2nd=7
 *
 * This code should work with GL3232, GL3233, GL3225, GL3226, GL3227 based products
 * even probably with GL3230, GL3231, GL3223, GL3224 based products.
 *
 * This code has been validated with a GL3232 device.
 *
 * VID:PID can be changed by the product vendor, to program these devices the custom
 * PID & VID should be provided as programmer params: vid= & pid=
 */

#include "programmer.h"
#include "spi.h"
#include "usb_device.h"

#include <libusb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define TRANSFER_TIMEOUT_MS     (200 + 800)

#define GL32XX_SPI_VID                  (0x05e3)

/* Tested devices to be filled here */
const struct dev_entry devs_gl32xx_spi[] = {
	{GL32XX_SPI_VID, 0x0764, OK, "Genesys Logic, Inc.", "GL3232"},
	{0}
};

/* Section 5.1: Command Block Wrapper (CBW) */
struct command_block_wrapper {
	uint8_t dCBWSignature[4];
	uint32_t dCBWTag;
	uint32_t dCBWDataTransferLength;
	uint8_t bmCBWFlags;
	uint8_t bCBWLUN;
	uint8_t bCBWCBLength;
	uint8_t CBWCB[16];
};

/* Section 5.2: Command Status Wrapper (CSW) */
struct command_status_wrapper {
	uint8_t dCSWSignature[4];
	uint32_t dCSWTag;
	uint32_t dCSWDataResidue;
	uint8_t bCSWStatus;
};

static int send_mass_storage_command(libusb_device_handle *handle, uint8_t endpoint,
		uint8_t *cdb, uint8_t cdb_len, uint8_t direction, int data_length,
		uint32_t *ret_tag)
{
	struct command_block_wrapper cbw;
	static uint32_t tag = 1;
	int r, size, i = 0;

	if (cdb == NULL)
		return -1;

	if (endpoint & LIBUSB_ENDPOINT_IN) {
		msg_perr("%s: cannot send command on IN endpoint\n", __func__);
		return -1;
	}

	memset(&cbw, 0, sizeof(cbw));
	cbw.dCBWSignature[0] = 'U';
	cbw.dCBWSignature[1] = 'S';
	cbw.dCBWSignature[2] = 'B';
	cbw.dCBWSignature[3] = 'C';
	*ret_tag = tag;
	cbw.dCBWTag = tag++;
	cbw.dCBWDataTransferLength = data_length;
	cbw.bmCBWFlags = direction;
	cbw.bCBWLUN = 0;
	cbw.bCBWCBLength = cdb_len;
	memcpy(cbw.CBWCB, cdb, cdb_len);

	/* The transfer length must always be exactly 31 bytes. */
	do {
		r = libusb_bulk_transfer(handle, endpoint, (unsigned char*)&cbw, 31,
						&size, TRANSFER_TIMEOUT_MS);
		if (r == LIBUSB_ERROR_PIPE) {
			libusb_clear_halt(handle, endpoint);
		}
		++i;
	} while ((r == LIBUSB_ERROR_PIPE) && (i<5));

	if (r != LIBUSB_SUCCESS) {
		msg_perr("%s: %s\n", __func__, libusb_strerror((enum libusb_error)r));
		return LIBUSB_ERROR(r);
	}

	return 0;
}

static int get_mass_storage_status(libusb_device_handle *handle, uint8_t endpoint,
				   uint32_t expected_tag)
{
	struct command_status_wrapper csw;
	int r, size, i = 0;

	/* The transfer length must always be exactly 31 bytes. */
	do {
		r = LIBUSB(libusb_bulk_transfer(handle, endpoint, (unsigned char*)&csw, 13,
						&size, TRANSFER_TIMEOUT_MS));
		if (r == LIBUSB_ERROR_PIPE) {
			libusb_clear_halt(handle, endpoint);
		}
		++i;
	} while ((r == LIBUSB_ERROR_PIPE) && (i<5));

	if (r != LIBUSB_SUCCESS) {
		msg_perr("%s: %s\n", __func__, libusb_strerror((enum libusb_error)r));
		return LIBUSB_ERROR(r);
	}

	if (size != 13) {
		msg_perr("%s: received %d bytes (expected 13)\n", __func__, size);
		return -1;
	}

	if (csw.dCSWTag != expected_tag) {
		msg_perr("%s: mismatched tags (expected %08X, received %08X)\n", __func__,
			expected_tag, csw.dCSWTag);
		return -1;
	}

	if (csw.bCSWStatus) {
		msg_perr("%s: Status %d DataResidue %d\n", __func__, csw.bCSWStatus,
			 csw.dCSWDataResidue);
		return -1;
	}

	return 0;
}

struct gl32xx_spi_data {
	struct usb_device *dev;
	uint8_t in_ep;
	uint8_t out_ep;
};

static struct gl32xx_spi_data *
	get_gl32xx_spi_data_from_context(const struct flashctx *flash)
{
	return (struct gl32xx_spi_data *)flash->mst->spi.data;
}

static int send_command(const struct flashctx *flash,
		unsigned int write_count,
		unsigned int read_count,
		const unsigned char *write_buffer,
		unsigned char *read_buffer)
{
	const struct gl32xx_spi_data *ctx_data = get_gl32xx_spi_data_from_context(flash);
	uint8_t cbd_write[6] = {0xf3, 0x02, 0, 0, 0, 0};
	uint8_t cbd_read[6] = {0xf3, 0x04, 0, 0, 0, 0};
	uint32_t tag;
	int r, size;

	/* Use a simpler command for a single-byte command */
	if (write_count == 1 && read_count == 0) {
		uint8_t cbd[6] = {0xf3, 0, 0, 0, write_buffer[0], 0};

		r = send_mass_storage_command(ctx_data->dev->handle, ctx_data->out_ep,
					      cbd, 6, 0, 0, &tag);
		if (r)
			return r;

		r = get_mass_storage_status(ctx_data->dev->handle, ctx_data->in_ep, tag);
		if (r)
			return r;

		return 0;
	}

	if (!write_count || write_count > 0xffff || read_count > 0xff)
		return -1;

	cbd_write[2] = (write_count >> 8) & 0xff;
	cbd_write[3] = write_count & 0xff;
	cbd_write[4] = (read_count >> 8) & 0xff;
	cbd_write[5] = read_count & 0xff;
	cbd_read[5] = read_count & 0xff;

	/* Write */
	r = send_mass_storage_command(ctx_data->dev->handle, ctx_data->out_ep, cbd_write,
				      6, 0, write_count, &tag);
	if (r)
		return r;

	r = LIBUSB(libusb_bulk_transfer(ctx_data->dev->handle, ctx_data->out_ep,
					(unsigned char *)write_buffer, write_count, &size,
					TRANSFER_TIMEOUT_MS));
	if (r)
		return r;

	r = get_mass_storage_status(ctx_data->dev->handle, ctx_data->in_ep, tag);
	if (r)
		return r;

	if (read_count == 0)
		return 0;

	/* Read */
	r = send_mass_storage_command(ctx_data->dev->handle, ctx_data->out_ep, cbd_read,
				      6, 0x80, read_count, &tag);
	if (r)
		return r;

	r = LIBUSB(libusb_bulk_transfer(ctx_data->dev->handle, ctx_data->in_ep, read_buffer,
					read_count, &size, TRANSFER_TIMEOUT_MS));
	if (r)
		return r;

	r = get_mass_storage_status(ctx_data->dev->handle, ctx_data->in_ep, tag);
	if (r)
		return r;

	return 0;
}

static int gl32xx_spi_write(struct flashctx *flash, const uint8_t *buf, unsigned int start,
			    unsigned int len)
{
	const struct gl32xx_spi_data *ctx_data = get_gl32xx_spi_data_from_context(flash);
	unsigned int remain = len, tx_size, pos = 0, address = start;
	uint8_t cbd_write[10] = {0xe5, 0x08, 0, 0, 0, 0, 0, 0, 0, 0};
	unsigned int maxpacketsize;
	uint32_t tag;
	int r, size;

	if (!len)
		return 0;

	r = libusb_get_max_packet_size(ctx_data->dev->device, ctx_data->out_ep);
	if (r < 0)
		return LIBUSB_ERROR(r);

	/* Limiting to 256bytes per command for write avoids write errors */
	maxpacketsize = min(256, r);

	while (remain) {
		tx_size = min(maxpacketsize, remain);

		cbd_write[2] = address >> 24;
		cbd_write[3] = (address >> 16) & 0xff;
		cbd_write[4] = (address >> 8) & 0xff;
		cbd_write[5] = address & 0xff;

		cbd_write[6] = tx_size >> 8;
		cbd_write[7] = tx_size & 0xff;

		r = send_mass_storage_command(ctx_data->dev->handle, ctx_data->out_ep, cbd_write,
					      10, 0, tx_size, &tag);
		if (r)
			return r;

		r = LIBUSB(libusb_bulk_transfer(ctx_data->dev->handle, ctx_data->out_ep,
						(unsigned char *)&buf[pos], tx_size, &size,
						TRANSFER_TIMEOUT_MS));
		if (r)
			return r;

		r = get_mass_storage_status(ctx_data->dev->handle, ctx_data->in_ep, tag);
		if (r)
			return r;

		remain -= tx_size;
		pos += tx_size;
		address += tx_size;
	}

	return 0;

}

static int gl32xx_spi_read(struct flashctx *flash, uint8_t *buf, unsigned int start,
			   unsigned int len)
{
	const struct gl32xx_spi_data *ctx_data = get_gl32xx_spi_data_from_context(flash);
	unsigned int remain = len, rx_size, pos = 0, address = start;
	uint8_t cbd_read[10] = {0xe4, 0x08, 0, 0, 0, 0, 0, 0, 0, 0};
	unsigned int maxpacketsize;
	uint32_t tag;
	int r, size;

	if (!len)
		return 0;

	r = libusb_get_max_packet_size(ctx_data->dev->device, ctx_data->in_ep);
	if (r < 0)
		return LIBUSB_ERROR(r);
	maxpacketsize = r;

	while (remain) {
		rx_size = min(maxpacketsize, remain);

		cbd_read[2] = address >> 24;
		cbd_read[3] = (address >> 16) & 0xff;
		cbd_read[4] = (address >> 8) & 0xff;
		cbd_read[5] = address & 0xff;

		cbd_read[6] = rx_size >> 8;
		cbd_read[7] = rx_size & 0xff;

		r = send_mass_storage_command(ctx_data->dev->handle, ctx_data->out_ep, cbd_read,
					      10, 0x80, rx_size, &tag);
		if (r)
			return r;

		r = LIBUSB(libusb_bulk_transfer(ctx_data->dev->handle, ctx_data->in_ep,
						&buf[pos], rx_size, &size,
						TRANSFER_TIMEOUT_MS));
		if (r)
			return r;

		r = get_mass_storage_status(ctx_data->dev->handle, ctx_data->in_ep, tag);
		if (r)
			return r;

		remain -= rx_size;
		pos += rx_size;
		address += rx_size;
	}

	return 0;

}

static const struct spi_master spi_master_gl32xx_spi = {
	.features       = SPI_MASTER_4BA,
	.max_data_read  = MAX_DATA_READ_UNLIMITED,
	.max_data_write = MAX_DATA_WRITE_UNLIMITED,
	.command        = send_command,
	.multicommand   = default_spi_send_multicommand,
	.read           = gl32xx_spi_read,
	.write_256      = gl32xx_spi_write,
	.write_aai      = default_spi_write_aai,
};

static int match_endpoint(struct libusb_endpoint_descriptor const *descriptor,
                          enum libusb_endpoint_direction direction)
{
	return (((descriptor->bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) ==
		 direction) &&
		((descriptor->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) ==
		 LIBUSB_TRANSFER_TYPE_BULK));
}

static int find_endpoints(struct usb_device *dev, uint8_t *in_ep, uint8_t *out_ep)
{
	int i;
	int in_count  = 0;
	int out_count = 0;

	for (i = 0; i < dev->interface_descriptor->bNumEndpoints; i++) {
		struct libusb_endpoint_descriptor const  *endpoint =
			&dev->interface_descriptor->endpoint[i];

		if (match_endpoint(endpoint, LIBUSB_ENDPOINT_IN)) {
			in_count++;
			*in_ep = endpoint->bEndpointAddress;
		} else if (match_endpoint(endpoint, LIBUSB_ENDPOINT_OUT)) {
			out_count++;
			*out_ep = endpoint->bEndpointAddress;
		}
	}

	if (in_count != 1 || out_count != 1) {
		msg_perr("Failed to find one IN and one OUT endpoint\n"
			 "        found %d IN and %d OUT endpoints\n",
			 in_count,
			 out_count);
		return 1;
	}

	msg_pdbg("Found IN  endpoint = 0x%02x\n", *in_ep);
	msg_pdbg("Found OUT endpoint = 0x%02x\n", *out_ep);

	return 0;
}

static int gl32xx_spi_shutdown(void * data)
{
	struct spi_master *spi_config = data;
	struct gl32xx_spi_data *ctx_data =
		(struct gl32xx_spi_data *)spi_config->data;

	usb_device_free(ctx_data->dev);
	libusb_exit(NULL);
	free(ctx_data);
	free(spi_config);

	return 0;
}

static void free_dev_list(struct usb_device **dev_lst)
{
	struct usb_device *dev = *dev_lst;
	/* free devices we don't care about */
	dev = dev->next;
	while (dev)
		dev = usb_device_free(dev);
}

/* This switches the MCU firmware to Flash programming mode */
static int gl32xx_spi_switch_program_mode(struct gl32xx_spi_data *data)
{
	uint8_t cbd[6] = {0xf3, 0x06, 0, 0, 0, 0};
	uint32_t tag;
	int r;

	r = send_mass_storage_command(data->dev->handle, data->out_ep, cbd, 6, 0, 0, &tag);
	if (r)
		return r;

	r = get_mass_storage_status(data->dev->handle, data->in_ep, tag);

	return r;
}

#define USB_CLASS_MASS_STORAGE          8

int gl32xx_spi_init(void)
{
	struct usb_match match;
	struct usb_device *current;
	struct usb_device *device = NULL;
	int found = 0;
	int ret;

	usb_match_init(&match);

	usb_match_value_default(&match.vid, GL32XX_SPI_VID);
	usb_match_value_default(&match.class, USB_CLASS_MASS_STORAGE);

	ret = LIBUSB(libusb_init(NULL));
	if (ret != 0) {
		msg_perr("GL32xx: libusb_init failed\n");
		return ret;
	}

	ret = usb_device_find(&match, &current);
	if (ret != 0) {
		msg_perr("GL32xx: Failed to find devices\n");
		return ret;
	}

	uint8_t in_endpoint  = 0;
	uint8_t out_endpoint = 0;
	while (current) {
		device = current;

		if (find_endpoints(device, &in_endpoint, &out_endpoint)) {
			msg_pdbg("GL32xx: Failed to find valid endpoints on device");
			usb_device_show(" ", current);
			current = usb_device_free(current);
			continue;
		}

		if (usb_device_claim(device)) {
			msg_pdbg("GL32xx: Failed to claim USB device");
			usb_device_show(" ", current);
			current = usb_device_free(current);
			continue;
		}

		found = 1;
		break;
	}

	if (!device || !found) {
		msg_perr("GL32xx: No usable device found.\n");
		return 1;
	}

	free_dev_list(&current);

	struct spi_master *spi_config = calloc(1, sizeof(struct spi_master));
	if (!spi_config) {
		msg_perr("Unable to allocate space for SPI master.\n");
		return SPI_GENERIC_ERROR;
	}
	struct gl32xx_spi_data *data = calloc(1, sizeof(struct gl32xx_spi_data));
	if (!data) {
		free(spi_config);
		msg_perr("Unable to allocate space for extra SPI master data.\n");
		return SPI_GENERIC_ERROR;
	}

	memcpy(spi_config, &spi_master_gl32xx_spi, sizeof(struct spi_master));

	data->dev = device;
	data->in_ep = in_endpoint;
	data->out_ep = out_endpoint;

	spi_config->data = data;

	gl32xx_spi_switch_program_mode(data);

	register_spi_master(spi_config);
	register_shutdown(gl32xx_spi_shutdown, spi_config);

	return 0;
}
