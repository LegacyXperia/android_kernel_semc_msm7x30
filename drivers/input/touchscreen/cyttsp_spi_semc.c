/* Source for:
 * Cypress TrueTouch(TM) Standard Product I2C touchscreen driver.
 * drivers/input/touchscreen/cyttsp_spi_semc.c
 *
 * Copyright (C) 2009, 2010 Cypress Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Cypress reserves the right to make changes without further notice
 * to the materials described herein. Cypress does not assume any
 * liability arising out of the application described herein.
 *
 * Contact Cypress Semiconductor at www.cypress.com
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/input/cyttsp_semc.h>
#include "cyttsp_core_semc.h"
#include <linux/mutex.h>
#include <linux/hrtimer.h>

#define DBG(x)

#define CY_SPI_WR_OP      0x00 /* r/~w */
#define CY_SPI_RD_OP      0x01
#define CY_SPI_CMD_BYTES  4
#define CY_SPI_SYNC_BYTES 2
#define CY_SPI_SYNC_BYTE  2
#define CY_SPI_SYNC_ACK1  0x62 /* from protocol v.2 */
#define CY_SPI_SYNC_ACK2  0x9D /* from protocol v.2 */
#define CY_SPI_SYNC_NACK  0x69
#define CY_SPI_MAX_PACKET 128
#define CY_SPI_DATA_SIZE  (CY_SPI_MAX_PACKET - CY_SPI_CMD_BYTES)
#define CY_SPI_DATA_BUF_SIZE (CY_SPI_CMD_BYTES + CY_SPI_DATA_SIZE)
#define CY_SPI_BITS_PER_WORD 8
#define SPI_TIMEOUT 100

struct cyttsp_spi {
	struct cyttsp_bus_ops ops;
	struct spi_device *spi_client;
	void *ttsp_client;
	u8 wr_buf[CY_SPI_DATA_BUF_SIZE];
	u8 rd_buf[CY_SPI_DATA_BUF_SIZE];
	struct hrtimer timer;
	struct mutex lock;
	atomic_t timeout;
};

static int cyttsp_spi_xfer_(u8 op, struct cyttsp_spi *ts_spi,
			    u8 reg, u8 *buf, int length)
{
	struct spi_message msg;
	struct spi_transfer xfer = { 0 };
	u8 *wr_buf = ts_spi->wr_buf;
	u8 *rd_buf = ts_spi->rd_buf;
	int retval;
	int i;
	DBG(printk(KERN_INFO "%s: Enter\n", __func__);)

	if (length > CY_SPI_DATA_SIZE) {
		printk(KERN_ERR "%s: length %d is too big.\n",
			__func__, length);
		return -EINVAL;
	}
	DBG(printk(KERN_INFO "%s: OP=%s length=%d\n", __func__,
		   op == CY_SPI_RD_OP ? "Read" : "Write", length);)

	wr_buf[0] = 0x00; /* header byte 0 */
	wr_buf[1] = 0xFF; /* header byte 1 */
	wr_buf[2] = reg;  /* reg index */
	wr_buf[3] = op;   /* r/~w */
	if (op == CY_SPI_WR_OP)
		memcpy(wr_buf + CY_SPI_CMD_BYTES, buf, length);
	DBG(
	if (op == CY_SPI_RD_OP)
		memset(rd_buf, CY_SPI_SYNC_NACK, CY_SPI_DATA_BUF_SIZE);)
	DBG(
	for (i = 0; i < (length + CY_SPI_CMD_BYTES); i++) {
		if ((op == CY_SPI_RD_OP) && (i < CY_SPI_CMD_BYTES))
			printk(KERN_INFO "%s: wr[%d]:0x%02x\n",
				__func__, i, wr_buf[i]);
		if (op == CY_SPI_WR_OP)
			printk(KERN_INFO "%s: wr[%d]:0x%02x\n",
				__func__, i, wr_buf[i]);
	})

	xfer.tx_buf = wr_buf;
	xfer.rx_buf = rd_buf;
	xfer.len = length + CY_SPI_CMD_BYTES;

	if ((op == CY_SPI_RD_OP) && (xfer.len < 32))
		xfer.len += 1;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	retval = spi_sync(ts_spi->spi_client, &msg);
	if (retval < 0) {
		printk(KERN_ERR "%s: spi_sync() error %d\n",
			__func__, retval);
		return retval;
	}
	if (op == CY_SPI_RD_OP) {
		DBG(
		for (i = 0; i < (length + CY_SPI_CMD_BYTES); i++)
			printk(KERN_INFO "%s: rd[%d]:0x%02x\n",
				__func__, i, rd_buf[i]);)

		for (i = 0; i < (CY_SPI_DATA_BUF_SIZE -
			(length + CY_SPI_SYNC_BYTES - 1)); i++) {
			if ((rd_buf[i] != CY_SPI_SYNC_ACK1) ||
				(rd_buf[i + 1] != CY_SPI_SYNC_ACK2)) {
				continue;
			}
			memcpy(buf, (rd_buf + i + CY_SPI_SYNC_BYTES), length);
			return 0;
		}
		DBG(printk(KERN_INFO "%s: byte sync error\n", __func__);)
		retval = -EAGAIN;
	} else {
		if ((rd_buf[CY_SPI_SYNC_BYTE] == CY_SPI_SYNC_ACK1) &&
			(rd_buf[CY_SPI_SYNC_BYTE+1] == CY_SPI_SYNC_ACK2))
			retval = 0;
		else
			retval = -EAGAIN;
	}
	return retval;
}

static enum hrtimer_restart watchdog(struct hrtimer *timer)
{
	struct cyttsp_spi *ts = container_of(timer,
						struct cyttsp_spi, timer);
	atomic_set(&ts->timeout, 1);
	pr_info("%s: cyttsp_spi: timeout\n", __func__);
	return HRTIMER_NORESTART;
}

static int cyttsp_spi_xfer(u8 op, struct cyttsp_spi *ts,
			    u8 reg, u8 *buf, int length)
{
	int rc;
	DBG(printk(KERN_INFO "%s: Enter\n", __func__);)

	mutex_lock(&ts->lock);
	atomic_set(&ts->timeout, 0);
	hrtimer_start(&ts->timer, ktime_set(0, SPI_TIMEOUT * 1000000L),
			HRTIMER_MODE_REL);
	do {
		rc = cyttsp_spi_xfer_(op, ts, reg, buf, length);
		if (!rc || rc != -EAGAIN)
			break;
	} while (!atomic_read(&ts->timeout));
	hrtimer_cancel(&ts->timer);
	mutex_unlock(&ts->lock);
	return rc;
}

static s32 ttsp_spi_read_block_data(void *handle, u8 addr,
				    u8 length, void *data)
{
	int retval;
	struct cyttsp_spi *ts = container_of(handle, struct cyttsp_spi, ops);

	DBG(printk(KERN_INFO "%s: Enter\n", __func__);)

	retval = cyttsp_spi_xfer(CY_SPI_RD_OP, ts, addr, data, length);
	if (retval < 0)
		printk(KERN_ERR "%s: ttsp_spi_read_block_data failed\n",
			__func__);

	return retval;
}

static s32 ttsp_spi_write_block_data(void *handle, u8 addr,
				     u8 length, const void *data)
{
	int retval;
	struct cyttsp_spi *ts = container_of(handle, struct cyttsp_spi, ops);

	DBG(printk(KERN_INFO "%s: Enter\n", __func__);)

	retval = cyttsp_spi_xfer(CY_SPI_WR_OP, ts, addr, (void *)data, length);
	if (retval < 0)
		printk(KERN_ERR "%s: ttsp_spi_write_block_data failed\n",
			__func__);
	return retval;
}

static s32 ttsp_spi_tch_ext(void *handle, void *values)
{
	int retval = 0;
	struct cyttsp_spi *ts = container_of(handle, struct cyttsp_spi, ops);

	DBG(printk(KERN_INFO "%s: Enter\n", __func__);)

	/* Add custom touch extension handling code here */
	/* set: retval < 0 for any returned system errors,
		retval = 0 if normal touch handling is required,
		retval > 0 if normal touch handling is *not* required */
	if (!ts || !values)
		retval = -EIO;

	return retval;
}

static int __devinit cyttsp_spi_probe(struct spi_device *spi)
{
	struct cyttsp_spi *ts_spi;
	int retval;
	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)

	/* Set up SPI*/
	spi->bits_per_word = CY_SPI_BITS_PER_WORD;
	spi->mode = SPI_MODE_0;
	retval = spi_setup(spi);
	if (retval < 0) {
		printk(KERN_ERR "%s: SPI setup error %d\n", __func__, retval);
		return retval;
	}
	ts_spi = kzalloc(sizeof(*ts_spi), GFP_KERNEL);
	if (ts_spi == NULL) {
		printk(KERN_ERR "%s: Error, kzalloc\n", __func__);
		retval = -ENOMEM;
		goto error_alloc_data_failed;
	}
	mutex_init(&ts_spi->lock);
	hrtimer_init(&ts_spi->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts_spi->timer.function = watchdog;
	ts_spi->spi_client = spi;
	dev_set_drvdata(&spi->dev, ts_spi);
	ts_spi->ops.write = ttsp_spi_write_block_data;
	ts_spi->ops.read = ttsp_spi_read_block_data;
	ts_spi->ops.ext = ttsp_spi_tch_ext;

	ts_spi->ttsp_client = cyttsp_core_init(&ts_spi->ops, &spi->dev);
	if (!ts_spi->ttsp_client)
		goto ttsp_core_err;
	printk(KERN_INFO "%s: Successful registration %s\n",
	       __func__, CY_SPI_NAME);

	return 0;

ttsp_core_err:
	kfree(ts_spi);
error_alloc_data_failed:
	return retval;
}

/* registered in driver struct */
static int __devexit cyttsp_spi_remove(struct spi_device *spi)
{
	struct cyttsp_spi *ts_spi = dev_get_drvdata(&spi->dev);
	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	cyttsp_core_release(ts_spi->ttsp_client);
	kfree(ts_spi);
	return 0;
}


static struct spi_driver cyttsp_spi_driver = {
	.driver = {
		.name = CY_SPI_NAME,
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = cyttsp_spi_probe,
	.remove = __devexit_p(cyttsp_spi_remove),
};

static int __init cyttsp_spi_init(void)
{
	int err;

	err = spi_register_driver(&cyttsp_spi_driver);
	printk(KERN_INFO "%s: Cypress TrueTouch(R) Standard Product SPI "
		"Touchscreen Driver (Built %s @ %s) returned %d\n",
		 __func__, __DATE__, __TIME__, err);

	return err;
}
module_init(cyttsp_spi_init);

static void __exit cyttsp_spi_exit(void)
{
	spi_unregister_driver(&cyttsp_spi_driver);
	printk(KERN_INFO "%s: module exit\n", __func__);
}
module_exit(cyttsp_spi_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product SPI driver");
MODULE_AUTHOR("Cypress");

