/*
 * fpga_spi.h
 *
 *  Created on: 2014年7月17日
 *      Author: root
 */

#ifndef FLASH_SPI_H_
#define FLASH_SPI_H_

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/crc7.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
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
#include <linux/io.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/gpio_flash.h>

#include <linux/gpio.h>
#include <asm/uaccess.h>

#include <linux/math64.h>


#include <linux/mod_devicetable.h>
//
//#include <linux/mtd/cfi.h>
//#include <linux/mtd/mtd.h>
//#include <linux/mtd/partitions.h>


#include <linux/spi/flash.h>

//#define SPI_MAX_CHUNK_SIZE    4092

#define FLASH_SPI_MAGIC 'k'

#define FLASH_SPI_CONF    _IOWR(FLASH_SPI_MAGIC,0x50,int)
#define FLASH_SPI_DONE    _IOR (FLASH_SPI_MAGIC,0x51,int)

#ifndef SPI_FLASH_AUTO_CS
#define FLASH_SPI_CS1     _IOWR (FLASH_SPI_MAGIC,0x52,int)
#endif

#define FLASH_SPI_ERASE   	_IO (FLASH_SPI_MAGIC,0x59)
#define FLASH_GPIO_MODE   	_IOWR (FLASH_SPI_MAGIC,0x54,int)
#define FLASH_SN_ERASE   	_IO (FLASH_SPI_MAGIC,0x5A)
#define FLASH_SN_READ   	_IOR (FLASH_SPI_MAGIC,0x5B,int)
#define FLASH_SN_WRITE   	_IOWR (FLASH_SPI_MAGIC,0x5C,int)

#define SPI_FLASH_SIZE 1024*1023*4
#define SPI_SN_SIZE 1024*4

typedef struct flash_sn{
	unsigned long length;
	unsigned char *data;
}flash_sn_t;

#endif /* FLASH_SPI_H_ */
