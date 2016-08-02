/*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <asm/uaccess.h>
#include <linux/delay.h>

/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/spidevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
#define SPIDEV_MAJOR			153	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */

static DECLARE_BITMAP(minors, N_SPI_MINORS);


/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *	is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY)


#define W25Q32_DeviceID 0xEF16
/* Flash opcodes. */
#define W25Q32_JedecDeviveID 0x9F /*Read identification */
#define W25Q32_ManufactDeviveID 0x90 /* Read identification */
#define W25Q32_ReadUniqueID 0x4B

#define W25Q32_CMD_ReadStatusReg1 0x05 /* Read Status Register instruction */
#define W25Q32_CMD_ReadStatusReg2 0x35 /* Read Status Register instruction */
#define W25Q32_CMD_WriteEnable 0x06 /*Write enable instruction */
#define W25Q32_CMD_WriteDisable 0x04 /*! Write to Memory Disable */



#define W25Q32_CMD_PageProgram 0x02 /* Write enable instruction */


#define W25Q32_CMD_BlockErase64 0xD8 /* Block 64k Erase instruction */
#define W25Q32_CMD_BlockErase32 0x52 /* Block 32k Erase instruction */
#define W25Q32_CMD_ChipErase 0xC7 /* Bulk Erase instruction */
#define W25Q32_CMD_SectorErase 0x20 /* Sector 4k Erase instruction */
#define W25Q32_CMD_EraseSuspend 0x75 /* Sector 4k Erase instruction */
#define W25Q32_CMD_EraseResume 0x7a /* Sector 4k Erase instruction */



#define W25Q32_CMD_High_Perform_Mode 0xa3
#define W25Q32_CMD_Conti_Read_Mode_Ret 0xff

#define W25Q32_WakeUp 0xAB
#define W25Q32_Power_Down 0xB9 /*Sector 4k Erase instruction */

#define W25Q32_CMD_ReadData 0x03 /* Read from Memory instruction */
#define W25Q32_CMD_FastRead 0x0b /* Read from Memory instruction */


#define W25Q32_DUMMY_BYTE 0xff //0xA5
#define W25Q32_SPI_PAGESIZE 0x100

/* Define max times to check status register before we give up. */
#define	MAX_READY_WAIT_JIFFIES	(40 * HZ)	/* M25P16 specs 40s max chip erase */
#define	MAX_CMD_SIZE		5
/* Status Register bits. */
#define	W25Q32_SR_WIP			1	/* Write in progress */
#define	W25Q32_SR_WEL			2	/* Write enable latch */


struct spidev_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex		buf_lock;
	unsigned		users;
	u8			*buffer;

    int result;
	u8  len;
	uint32_t  addr;
	uint32_t  const_addr;
	u8   cmd;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

/*-------------------------------------------------------------------------*/

/*
 * We can't use the standard synchronous wrappers for file I/O; we
 * need to protect against async removal of the underlying spi_device.
 */
static void spidev_complete(void *arg)
{
	complete(arg);
}

static ssize_t
spidev_sync(struct spidev_data *spidev, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = spidev_complete;
	message->context = &done;

	spin_lock_irq(&spidev->spi_lock);
	if (spidev->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(spidev->spi, message);
	spin_unlock_irq(&spidev->spi_lock);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
		if (status == 0)
			status = message->actual_length;
	}
	return status;
}

static inline ssize_t
spidev_sync_write(struct spidev_data *spidev, size_t len)
{
	struct spi_transfer	t = {
			.tx_buf		= spidev->buffer,
			.len		= len,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spidev_sync(spidev, &m);
}

static inline ssize_t
spidev_sync_read(struct spidev_data *spidev, size_t len)
{
	struct spi_transfer	t = {
			.rx_buf		= spidev->buffer,
			.len		= len,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spidev_sync(spidev, &m);
}

/*-------------------------------------------------------------------------*/

/* Read-only message with current device setup */
static ssize_t
spidev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct spidev_data	*spidev;
	ssize_t			status = 0;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	spidev = filp->private_data;

	mutex_lock(&spidev->buf_lock);
	status = spidev_sync_read(spidev, count);
	if (status > 0) {
		unsigned long	missing;

		missing = copy_to_user(buf, spidev->buffer, status);
		if (missing == status)
			status = -EFAULT;
		else
			status = status - missing;
	}
	mutex_unlock(&spidev->buf_lock);

	return status;
}

/* Write-only message with current device setup */
static ssize_t
spidev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct spidev_data	*spidev;
	ssize_t			status = 0;
	unsigned long		missing;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	spidev = filp->private_data;

	mutex_lock(&spidev->buf_lock);
	missing = copy_from_user(spidev->buffer, buf, count);
	if (missing == 0) {
		status = spidev_sync_write(spidev, count);
	} else
		status = -EFAULT;
	mutex_unlock(&spidev->buf_lock);

	return status;
}

static int spidev_message(struct spidev_data *spidev,
		struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
	struct spi_message	msg;
	struct spi_transfer	*k_xfers;
	struct spi_transfer	*k_tmp;
	struct spi_ioc_transfer *u_tmp;
	unsigned		n, total;
	u8			*buf;
	int			status = -EFAULT;

	spi_message_init(&msg);
	k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
	if (k_xfers == NULL)
		return -ENOMEM;

	/* Construct spi_message, copying any tx data to bounce buffer.
	 * We walk the array of user-provided transfers, using each one
	 * to initialize a kernel version of the same transfer.
	 */
	buf = spidev->buffer;
	total = 0;
	for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
			n;
			n--, k_tmp++, u_tmp++) {
		k_tmp->len = u_tmp->len;

		total += k_tmp->len;
		if (total > bufsiz) {
			status = -EMSGSIZE;
			goto done;
		}

		if (u_tmp->rx_buf) {
			k_tmp->rx_buf = buf;
			if (!access_ok(VERIFY_WRITE, (u8 __user *)
						(uintptr_t) u_tmp->rx_buf,
						u_tmp->len))
				goto done;
		}
		if (u_tmp->tx_buf) {
			k_tmp->tx_buf = buf;
			if (copy_from_user(buf, (const u8 __user *)
						(uintptr_t) u_tmp->tx_buf,
					u_tmp->len))
				goto done;
		}
		buf += k_tmp->len;

		k_tmp->cs_change = !!u_tmp->cs_change;
		k_tmp->bits_per_word = u_tmp->bits_per_word;
		k_tmp->delay_usecs = u_tmp->delay_usecs;
		k_tmp->speed_hz = u_tmp->speed_hz;
#ifdef VERBOSE
		dev_dbg(&spidev->spi->dev,
			"  xfer len %zd %s%s%s%dbits %u usec %uHz\n",
			u_tmp->len,
			u_tmp->rx_buf ? "rx " : "",
			u_tmp->tx_buf ? "tx " : "",
			u_tmp->cs_change ? "cs " : "",
			u_tmp->bits_per_word ? : spidev->spi->bits_per_word,
			u_tmp->delay_usecs,
			u_tmp->speed_hz ? : spidev->spi->max_speed_hz);
#endif
		spi_message_add_tail(k_tmp, &msg);
	}

	status = spidev_sync(spidev, &msg);
	if (status < 0)
		goto done;

	/* copy any rx data out of bounce buffer */
	buf = spidev->buffer;
	for (n = n_xfers, u_tmp = u_xfers; n; n--, u_tmp++) {
		if (u_tmp->rx_buf) {
			if (__copy_to_user((u8 __user *)
					(uintptr_t) u_tmp->rx_buf, buf,
					u_tmp->len)) {
				status = -EFAULT;
				goto done;
			}
		}
		buf += u_tmp->len;
	}
	status = total;

done:
	kfree(k_xfers);
	return status;
}
/**********************************************************************/
//The return value is readed value for success, else a negative errno status code
static int
w25q32_read_sr(struct spidev_data *spidev)
{
	ssize_t retval;
	u8 code = W25Q32_CMD_ReadStatusReg1;//0x05
	u8 val;

	retval = spi_write_then_read(spidev->spi, &code, 1, &val, 1);

	if (retval < 0) {
		dev_err(&spidev->spi->dev, "error %d reading SR\n",
				(int) retval);
		return retval;
	}

	return val;
}

/*
 * Write status register 1 byte
 * Returns negative if error occurred.
 */
static int
w25q32_write_sr(struct spidev_data *spidev, u8 val)
{
//	flash->command[0] = OPCODE_WRSR;
//	flash->command[1] = val;
//
//	return spi_write(flash->spi, flash->command, 2);
	return 0;
}

/*
 * Set write enable latch with Write Enable command.
 * Returns negative if error occurred.
 */
static inline int
w25q32_write_enable(struct spidev_data *spidev)
{
	u8	code = W25Q32_CMD_WriteEnable;

	return spi_write_then_read(spidev->spi, &code, 1, NULL, 0);
}

/*
 * Send write disble instruction to the chip.
 */
static inline int
w25q32_write_disable(struct spidev_data *spidev)
{
	u8	code = W25Q32_CMD_WriteDisable;

	return spi_write_then_read(spidev->spi, &code, 1, NULL, 0);
}


/*
 * Service routine to read status register until ready, or timeout occurs.
 * Returns non-zero if error.
 */
static int
w25q32_wait_till_ready(struct spidev_data *spidev)
{
	unsigned long deadline;
	int sr;

	deadline = jiffies + MAX_READY_WAIT_JIFFIES;

	do {
		if ((sr = w25q32_read_sr(spidev)) < 0)
			break;
		else if (!(sr & W25Q32_SR_WIP))
			return 0;

		//cond_resched();

	} while (!time_after_eq(jiffies, deadline));

	return 1;
}

static int
w25q32_wait_null(struct spidev_data *spidev)
{
    uint8_t limit = 5;
    uint8_t ret=0;
    /* wait BUSY bit clear */
    ret=w25q32_read_sr(spidev);
    while(((ret & 0x01) == W25Q32_SR_WIP) && (limit != 0))
     {
        limit--;
        mdelay(100);
        ret=w25q32_read_sr(spidev);
        printk("w25q32_wait_null status register 0x%0x\n",ret);
    }
    if (limit == 0) {
        printk("!!!!w25q_wait_null:time out!\n");
        return -EBUSY;
    }
    else
        return 0;
}


/*
 * when you call this function,
 * the w25q->cmd, w25q->len(receive len)
 * w25q->buf(kzalloc receive) and w25q->addr are OK
 *
 */
static int
w25q32_read_data(struct spidev_data *spidev)
{
    int ret = 0;
    struct spi_message message;
    struct spi_transfer    x[(spidev->len + 4) * 2];
    u8 i = 0, rx = 0, dumy_value = 0xff, tx_buff[4] = {0};

    w25q32_write_enable(spidev);    //SET WEL
    ret = w25q32_wait_null(spidev);
    if (ret != 0) {
        printk("!!!!chip_erase: wait null err!\n");
        return ret;
    }
    if((w25q32_read_sr(spidev) & 0x02) != 0x02) {
        printk("!!!!state register write able is 0\n");
        return -EBUSY;    //disable write
    }

    printk("cmd = 0x%x, addr = 0x%x\n", spidev->cmd, spidev->addr);
    tx_buff[0] = spidev->cmd;
    tx_buff[1] = ((uint8_t)(spidev->addr >> 16));
    tx_buff[2] = ((uint8_t)(spidev->addr >> 8));
    tx_buff[3] = ((uint8_t)(spidev->addr));

    spi_message_init(&message);
    memset(x, 0, sizeof x);
    for (i = 0; i < 8; i++) {    //cmd
        x[i].len = 1;
        spi_message_add_tail(&x[i], &message);
        if ((i % 2) == 0) {
            x[i].tx_buf = &tx_buff[i / 2];
        } else {
            x[i].rx_buf = &rx;
        }
    }
    for (i = 8; i < (spidev->len + 4) * 2; i++) {
        x[i].len = 1;
        spi_message_add_tail(&x[i], &message);
        if ((i % 2) == 0) {
            x[i].tx_buf = &dumy_value;
        } else {
            x[i].rx_buf = spidev->buffer++;
        }
    }
    /* do the i/o */
    return spi_sync(spidev->spi, &message);
}



static int
w25q32_read_id(struct spidev_data *spidev)
{

    int ret = 0;
    u8 *buf_start;

    printk("w25q32_read_id\n");
    spidev->len = 3;
    spidev->addr = 0;
    spidev->cmd = W25Q32_JedecDeviveID;
    buf_start = spidev->buffer = kzalloc(spidev->len, GFP_KERNEL);
    if (!buf_start) {
        printk("!!!!kzalloc is error\n");
        return -ENOMEM;
    }
    ret = w25q32_read_data(spidev);
    spidev->buffer = buf_start;
//    w25q->result = *w25q->buf << 8;
//    w25q->buf++;
//    w25q->result |= *w25q->buf;

    printk( "Manufacture ID: 0x%x\n", spidev->buffer[0] );
    printk( "Device ID: 0x%x\n", spidev->buffer[1] | spidev->buffer[2] << 8 );
    spidev->result = spidev->buffer[2] << 16 | spidev->buffer[1] << 8 | spidev->buffer[0];;
    kfree(buf_start);
    if (ret != 0) {
        printk("!!!!w25q32_read_id: w25q_read_data error!\n");
        return ret;
    }
    printk("w25q32_read_id: read id OK\n");
    return ret;



//    int ret = 0;
//    uint8_t *buf_start;
//
//    printk("*****w25q32_read_data******\n");
//
//    ret=w25q32_write_enable(spidev);    //OPCODE_WREN
//    if(ret!=0)
//    {
//        printk("!!!!chip_write enable failed!\n");
//        return ret;
//
//    }
//    ret = w25q32_wait_null(spidev);    //Check Status Register RDY0/BSY1
//    if (ret != 0)
//    {
//        printk("!!!!at26df_wait_null failed!\n");
//        return ret;
//    }
//    ret=w25q32_read_sr(spidev);
//    if(( ret& 0x02) != 0x02) //OPCODE_RDSR ,and Check Status Register WEL(1 enable)
//    {
//        printk("!!!!state register write able is 0,and status register 0x%0x\n",ret);
//        return -EBUSY;    //disable write
//    }
//
//    // Read JEDEC ID (9Fh)
//    spidev->len = 3;
//    spidev->addr = 0;
//    spidev->cmd = W25Q32_JedecDeviveID;//0X9F
//    buf_start = spidev->buffer=(uint8_t *)kzalloc(spidev->len, GFP_KERNEL);
//    if (!buf_start)
//    {
//        printk("!!!!kzalloc is error\n");
//        return -ENOMEM;
//    }
//    ret=spi_write_then_read(spidev->spi,&spidev->cmd, 1, spidev->buffer, spidev->len );
//    printk( "Manufacture ID: 0x%x\n", spidev->buffer[0] );
//    printk( "Device ID: 0x%x\n", spidev->buffer[1] | spidev->buffer[2] << 8 );
//    spidev->result = spidev->buffer[2] << 16 | spidev->buffer[1] << 8 | spidev->buffer[0];;
//
//    kfree(buf_start);
//    if (ret != 0)
//    {
//        printk("!!!!w25q32_read_id: w25q32_read_data error!\n");
//        return ret;
//    }
//    printk("w25q32_read_id: read id OK\n");
//    return ret;


}



static int
w25q32_chip_erase(struct spidev_data *spidev)
{
    int ret = 0;

    printk("w25q_chip_erase\n");
    w25q32_write_enable(spidev);    //SET WEL
    ret = w25q32_wait_null(spidev);
    if (ret != 0)
    {
        printk("!!!!chip_erase: wait null err!\n");
        return ret;
    }
    if((w25q32_read_sr(spidev)& 0x02) != 0x02)
    {
        printk("!!!!state register write able is 0\n");
        return -EBUSY;    //disable write
    }
    /* Set up command buffer. */
	spidev->cmd=W25Q32_CMD_ChipErase;

	return spi_write(spidev->spi, spidev->cmd, 1);

//    spi_w8r8(spidev->spi, spidev->cmd);
//    return w25q32_wait_null(spidev);
}


static int
w25q32_write_data(struct spidev_data *spidev)
{
    int ret = 0;
    u8 i = 0, rx = 0;
    struct spi_message message;
    struct spi_transfer    x[(spidev->len + 4) * 2];

    w25q32_write_enable(spidev);    //SET WEL
    ret = w25q32_wait_null(spidev);
    if (ret != 0) {
        printk("!!!!w25q32_write_date: wait null err!\n");
        return ret;
    }
    if((w25q32_read_sr(spidev) & 0x02) != 0x02) {
        printk("!!!!state register write able is 0\n");
        return -EBUSY;    //disable write
    }

    printk("cmd = 0x%x, addr = 0x%x\n", spidev->cmd, spidev->addr);
    spidev->buffer[0] = spidev->cmd;
    spidev->buffer[1] = ((u8)(spidev->addr >> 16));
    spidev->buffer[2] = ((u8)(spidev->addr >> 8));
    spidev->buffer[3] = ((u8)spidev->addr);

    spi_message_init(&message);
    memset(x, 0, sizeof x);
    for (i = 0; i < (spidev->len + 4) * 2; i++) {
        x[i].len = 1;
        spi_message_add_tail(&x[i], &message);
        if ((i % 2) == 0) {
            x[i].tx_buf = spidev->buffer++;
        } else {
            x[i].rx_buf = &rx;
        }
    }
    /* do the i/o */
    ret = spi_sync(spidev->spi, &message);
    if (ret != 0) {
        printk("!!!!w25q_write_data: spi_sync() error!");
        return ret;
    }
    ret = w25q32_wait_null(spidev);
    if (ret != 0)
        printk("!!!!w25q32_write_data: w25q32_wait_null() error!");
    return ret;
}


#define W25Q32_SECTOR_MAX   0xff
#define W25Q32_ONE_SECTOR_ADDR 0xff
#define W25Q32_32K_BLOCK_MAX 0xff
#define W25Q32_32K_BLOCK_ADDR  0xff
#define W25Q32_BLOCK_MAX 0xff
#define  W25Q32_ONE_BLOCK_ADDR 0xff

static int
w25q32_erase(struct spidev_data *spidev, u32 num, unsigned int cmd)
{
    int ret = 0;
    u8 *buf_start;

    switch(cmd)
    {
    case W25Q32_CMD_SectorErase:
        printk("sector erase cmd\n");
        if (num > W25Q32_SECTOR_MAX)
        {
            printk("!!!!sector max is over\n");
            return -EFAULT;
        }
        spidev->const_addr = num * W25Q32_ONE_SECTOR_ADDR;
        spidev->cmd = W25Q32_CMD_SectorErase;
        break;
    case W25Q32_CMD_BlockErase32:
        printk("32K  block erase cmd\n");
        if (num > W25Q32_32K_BLOCK_MAX)
        {
            printk("!!!!half block max is over\n");
            return -EFAULT;
        }
        spidev->const_addr = num * W25Q32_32K_BLOCK_ADDR;
        spidev->cmd = W25Q32_CMD_BlockErase32;
        break;
    case W25Q32_CMD_BlockErase64:
        printk("64K block erase cmd\n");
        if (num > W25Q32_BLOCK_MAX)
        {
            printk("!!!!block max is over\n");
            return -EFAULT;
        }
        spidev->const_addr = num * W25Q32_ONE_BLOCK_ADDR;
        spidev->cmd = W25Q32_CMD_BlockErase64;
        break;
    }

    printk("w25q->const_addr = 0x%x\n", spidev->const_addr);
    spidev->len = 0;
    buf_start = spidev->buffer = kzalloc(spidev->len + 4, GFP_KERNEL);
    if (!buf_start) {
        printk("!!!!kzalloc is error\n");
        return -ENOMEM;
    }
    spidev->addr = spidev->const_addr;
    //ret = w25q_write_data(spidev);
    kfree(buf_start);
    if (ret != 0) {
        printk("!!!!w25q32_erase: spi write err!\n");
        return ret;
    }
    printk("w25q32_erase: erase OK\n");
    return ret;
}
#define W25Q32_BUF_LEN                4096
#define W25Q32_PAGE_NUM                256



static ssize_t
w25q32_read1(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    u8 *buf_start, *buf_tmp, *w25q32_buf;
    struct spidev_data *spidev = file->private_data;
    u32 buf_size = 0, read_len = 0, page_num = W25Q32_PAGE_NUM;

    printk("@@@@w25q read start\n");
    buf_start = buf_tmp = kzalloc(W25Q32_BUF_LEN, GFP_KERNEL);
    w25q32_buf = spidev->buffer = kzalloc(page_num, GFP_KERNEL);
    if (!buf_start || !w25q32_buf ) {
        printk("!!!!kzalloc error!\n");
        return -ENOMEM;
    }
    ret = mutex_lock_interruptible(&spidev->buf_lock);
    if (ret) {
        printk("!!!!mutex lock error!\n");
        goto exit_kfree;
    }
    read_len = W25Q32_BUF_LEN;
    buf_size = min(count, read_len);
    read_len = buf_size;

    spidev->cmd = W25Q32_CMD_ReadData;
    spidev->addr = spidev->const_addr;
    printk("spidev->addr = 0x%x\n", spidev->addr);
    while (buf_size) {
        spidev->buffer = w25q32_buf;
        spidev->len = min(buf_size, page_num);
        ret = w25q32_read_data(spidev);
        if (ret != 0) {
            goto exit_lock;
        }
        memcpy(buf_tmp, w25q32_buf, spidev->len);
        buf_tmp += spidev->len;
        buf_size -= spidev->len;
        spidev->addr += spidev->len;
    }
    ret = copy_to_user(user_buf, buf_start, read_len);
    ret = read_len -ret;

exit_lock:
    mutex_unlock(&spidev->buf_lock);
exit_kfree:
    kfree(buf_start);
    kfree(w25q32_buf);
    printk("w25q32 read stop, ret = %d\n", ret);
    return ret;
}


static ssize_t
w25q32_write1(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    u8 *buf_start, *buf_tmp, *w25q32_buf;
    struct spidev_data *spidev= file->private_data;
    u32 buf_size = 0, page_num = W25Q32_PAGE_NUM, len = 0;

    printk("@@@@w25q32 write start\n");
    buf_start = buf_tmp = kzalloc(W25Q32_BUF_LEN, GFP_KERNEL);
    w25q32_buf = spidev->buffer = kzalloc(page_num + 4, GFP_KERNEL);
    if (!buf_start || !w25q32_buf) {
        printk("!!!!kzalloc error!\n");
        return -ENOMEM;
    }
    ret = mutex_lock_interruptible(&spidev->buf_lock);
    if (ret) {
        printk("!!!!mutex lock error!");
        goto exit_kfree;
    }
    len = W25Q32_BUF_LEN;
    buf_size = min(count, len);
    if (copy_from_user(buf_tmp, user_buf, buf_size)) {
        printk("!!!!copy_from_user() error!\n");
        ret = -EFAULT;
        goto exit_lock;
    }
    printk("spidev->const_addr = 0x%x\n", spidev->const_addr);
    buf_tmp = buf_start;
    spidev->cmd = W25Q32_CMD_PageProgram;
    spidev->addr = spidev->const_addr;
    while(buf_size) {
        spidev->buffer = w25q32_buf;
        spidev->len = min(buf_size, page_num);
        memcpy(spidev->buffer + 4, buf_tmp, spidev->len);
        ret = w25q32_write_data(spidev);
        if (ret != 0) {
            break;
        }
        buf_tmp += spidev->len;
        spidev->addr += spidev->len;
        buf_size -= spidev->len;
    }

exit_lock:
    mutex_unlock(&spidev->buf_lock);
exit_kfree:
    kfree(buf_start);
    kfree(w25q32_buf);
    if (ret != 0)
        printk("!!!!w25q32 write error!\n");
    else
        printk("w25q32 write success\n");
    return ret;
}

/******************************************************************************/
static long
spidev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int			err = 0;
	int			retval = 0;
	struct spidev_data	*spidev;
	struct spi_device	*spi;
	u32			tmp;
	unsigned		n_ioc;
	struct spi_ioc_transfer	*ioc;
    printk("*****     spidev_ioctl       *****\n");
//	/* Check type and command number */
//	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
//		return -ENOTTY;
//    printk("*****     spidev_ioctl   check type pass    *****\n");
//	/* Check access direction once here; don't repeat below.
//	 * IOC_DIR is from the user perspective, while access_ok is
//	 * from the kernel perspective; so they look reversed.
//	 */
//
//	if (_IOC_DIR(cmd) & _IOC_READ)
//		err = !access_ok(VERIFY_WRITE,
//				(void __user *)arg, _IOC_SIZE(cmd));
//	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
//		err = !access_ok(VERIFY_READ,
//				(void __user *)arg, _IOC_SIZE(cmd));
//	if (err)
//		return -EFAULT;
//    printk("*****     spidev_ioctl   check access pass    *****\n");
	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	spidev = filp->private_data;
	spin_lock_irq(&spidev->spi_lock);
	spi = spi_dev_get(spidev->spi);
	spin_unlock_irq(&spidev->spi_lock);

	if (spi == NULL)
		return -ESHUTDOWN;

	/* use the buffer lock here for triple duty:
	 *  - prevent I/O (from us) so calling spi_setup() is safe;
	 *  - prevent concurrent SPI_IOC_WR_* from morphing
	 *    data fields while SPI_IOC_RD_* reads them;
	 *  - SPI_IOC_MESSAGE needs the buffer locked "normally".
	 */
	mutex_lock(&spidev->buf_lock);

	switch (cmd) {
	/* read requests */
	case SPI_IOC_RD_MODE:
		retval = __put_user(spi->mode & SPI_MODE_MASK,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_LSB_FIRST:
		retval = __put_user((spi->mode & SPI_LSB_FIRST) ?  1 : 0,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_BITS_PER_WORD:
		retval = __put_user(spi->bits_per_word, (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MAX_SPEED_HZ:
		retval = __put_user(spi->max_speed_hz, (__u32 __user *)arg);
		break;

	/* write requests */
	case SPI_IOC_WR_MODE:
		retval = __get_user(tmp, (u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->mode;

			if (tmp & ~SPI_MODE_MASK) {
				retval = -EINVAL;
				break;
			}

			tmp |= spi->mode & ~SPI_MODE_MASK;
			spi->mode = (u8)tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "spi mode %02x\n", tmp);
		}
		break;
	case SPI_IOC_WR_LSB_FIRST:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->mode;

			if (tmp)
				spi->mode |= SPI_LSB_FIRST;
			else
				spi->mode &= ~SPI_LSB_FIRST;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "%csb first\n",
						tmp ? 'l' : 'm');
		}
		break;
	case SPI_IOC_WR_BITS_PER_WORD:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->bits_per_word;

			spi->bits_per_word = tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->bits_per_word = save;
			else
				dev_dbg(&spi->dev, "%d bits per word\n", tmp);
		}
		break;
	case SPI_IOC_WR_MAX_SPEED_HZ:
		retval = __get_user(tmp, (__u32 __user *)arg);
		if (retval == 0) {
			u32	save = spi->max_speed_hz;

			spi->max_speed_hz = tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->max_speed_hz = save;
			else
				dev_dbg(&spi->dev, "%d Hz (max)\n", tmp);
		}
		break;
//add by moonic
	case W25Q32_JedecDeviveID:
        printk("***** ioctl go Read device ID *****\n");
        retval = w25q32_read_id(spidev);
		if (retval == 0)
		{
            printk("Read ID sucess\n");
            __put_user(spidev->result, (__u32 __user *)arg);
            //put_user(spidev->result, *arg);
		}

		break;



	default:
		/* segmented and/or full-duplex I/O request */
		if (_IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
				|| _IOC_DIR(cmd) != _IOC_WRITE) {
			retval = -ENOTTY;
			break;
		}

		tmp = _IOC_SIZE(cmd);
		if ((tmp % sizeof(struct spi_ioc_transfer)) != 0) {
			retval = -EINVAL;
			break;
		}
		n_ioc = tmp / sizeof(struct spi_ioc_transfer);
		if (n_ioc == 0)
			break;

		/* copy into scratch area */
		ioc = kmalloc(tmp, GFP_KERNEL);
		if (!ioc) {
			retval = -ENOMEM;
			break;
		}
		if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
			kfree(ioc);
			retval = -EFAULT;
			break;
		}

		/* translate to spi_message, execute */
		retval = spidev_message(spidev, ioc, n_ioc);
		kfree(ioc);
		break;
	}

	mutex_unlock(&spidev->buf_lock);
	spi_dev_put(spi);
	return retval;
}

#ifdef CONFIG_COMPAT
static long
spidev_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return spidev_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define spidev_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int spidev_open(struct inode *inode, struct file *filp)
{
	struct spidev_data	*spidev;
	int			status = -ENXIO;

	mutex_lock(&device_list_lock);

	list_for_each_entry(spidev, &device_list, device_entry) {
		if (spidev->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status == 0) {
		if (!spidev->buffer) {
			spidev->buffer = kmalloc(bufsiz, GFP_KERNEL);
			if (!spidev->buffer) {
				dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		if (status == 0) {
			spidev->users++;
			filp->private_data = spidev;
			nonseekable_open(inode, filp);
		}
	} else
		pr_debug("spidev: nothing for minor %d\n", iminor(inode));

	mutex_unlock(&device_list_lock);
	return status;
}

static int spidev_release(struct inode *inode, struct file *filp)
{
	struct spidev_data	*spidev;
	int			status = 0;

	mutex_lock(&device_list_lock);
	spidev = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	spidev->users--;
	if (!spidev->users) {
		int		dofree;

		kfree(spidev->buffer);
		spidev->buffer = NULL;

		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&spidev->spi_lock);
		dofree = (spidev->spi == NULL);
		spin_unlock_irq(&spidev->spi_lock);

		if (dofree)
			kfree(spidev);
	}
	mutex_unlock(&device_list_lock);

	return status;
}

static const struct file_operations spidev_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.write =	spidev_write,
	.read =		spidev_read,
	.unlocked_ioctl = spidev_ioctl,
	.compat_ioctl = spidev_compat_ioctl,
	.open =		spidev_open,
	.release =	spidev_release,
	.llseek =	no_llseek,
};

/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *spidev_class;

/*-------------------------------------------------------------------------*/

static int __devinit spidev_probe(struct spi_device *spi)
{
	struct spidev_data	*spidev;
	int			status;
	unsigned long		minor;

	printk(KERN_ERR"*************************************************\n");
	/* Allocate driver data */
	spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);
	if (!spidev)
		return -ENOMEM;

	/* Initialize the driver data */
	spidev->spi = spi;
	spin_lock_init(&spidev->spi_lock);
	mutex_init(&spidev->buf_lock);

	INIT_LIST_HEAD(&spidev->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		spidev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(spidev_class, &spi->dev, spidev->devt,
				    spidev, "spidev%d.%d",
				    spi->master->bus_num, spi->chip_select);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&spidev->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	if (status == 0)
	{
        u8 buf[4]={0};
        u8 cmd =  0x9F;
        spi_set_drvdata(spi, spidev);

        /*
        spi_write_then_read( spidev->spi,&cmd, 1, buf, 3 );

        printk( "Manufacture ID: 0x%x\n", buf[0] );
        printk( "Device ID: 0x%x\n", buf[1] | buf[2] << 8 );
        */
    }

	else
		kfree(spidev);



	return status;
}

static int __devexit spidev_remove(struct spi_device *spi)
{
	struct spidev_data	*spidev = spi_get_drvdata(spi);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&spidev->spi_lock);
	spidev->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&spidev->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&spidev->device_entry);
	device_destroy(spidev_class, spidev->devt);
	clear_bit(MINOR(spidev->devt), minors);
	if (spidev->users == 0)
		kfree(spidev);
	mutex_unlock(&device_list_lock);

	return 0;
}

static struct spi_driver spidev_spi_driver = {
	.driver = {
		.name =		"spidev",
		.owner =	THIS_MODULE,
	},
	.probe =	spidev_probe,
	.remove =	__devexit_p(spidev_remove),

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.  The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */
};

/*-------------------------------------------------------------------------*/

static int __init spidev_init(void)
{
	int status;

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, "spi", &spidev_fops);
	if (status < 0)
		return status;

	spidev_class = class_create(THIS_MODULE, "spidev");
	if (IS_ERR(spidev_class)) {
		unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
		return PTR_ERR(spidev_class);
	}

	status = spi_register_driver(&spidev_spi_driver);
	if (status < 0) {
		class_destroy(spidev_class);
		unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
	}
	return status;
}
module_init(spidev_init);

static void __exit spidev_exit(void)
{
	spi_unregister_driver(&spidev_spi_driver);
	class_destroy(spidev_class);
	unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
}
module_exit(spidev_exit);

MODULE_AUTHOR("Andrea Paterniani, <a.paterniani@swapp-eng.it>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spidev");
