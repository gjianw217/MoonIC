/*
 * flash_spi.c
 *
 *  Created on: 2016年5月15日
 *  Author: Jianwei@Guo
 */

#include "spi-w25q32.h"

#define SPIDEV_MAJOR			154	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */

static DECLARE_BITMAP(minors, N_SPI_MINORS);
static struct class *flash_spi_class;

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
#define SPI_MODE_MASK	(SPI_MODE_3 | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY)



/* Flash opcodes. */
#define	OPCODE_WREN		0x06	/* Write enable */
#define	OPCODE_RDSR		0x05	/* Read status register */
#define	OPCODE_WRSR		0x01	/* Write status register 1 byte */
#define	OPCODE_NORM_READ	0x03	/* Read data bytes (low frequency) */
#define	OPCODE_FAST_READ	0x0b	/* Read data bytes (high frequency) */
#define	OPCODE_PP		0x02	/* Page program (up to 256 bytes) */
#define	OPCODE_BE_4K		0x20	/* Erase 4KiB block */
#define	OPCODE_BE_32K		0x52	/* Erase 32KiB block */
#define	OPCODE_CHIP_ERASE	0xc7	/* Erase whole flash chip */
#define	OPCODE_SE		0xd8	/* Sector erase (usually 64KiB) */
#define	OPCODE_RDID		0x9f	/* Read JEDEC ID */

#define	OPCODE_WRDI		0x04

#define	OPCODE_BRWR		0x17
#define	OPCODE_AAI_WP		0xad
#define	OPCODE_BP		0x02

/* Status Register bits. */
#define	SR_WIP			1	/* Write in progress */
#define	SR_WEL			2	/* Write enable latch */
/* meaning of other SR_* bits may differ between vendors */
#define	SR_BP0			4	/* Block protect 0 */
#define	SR_BP1			8	/* Block protect 1 */
#define	SR_BP2			0x10	/* Block protect 2 */
#define	SR_SRWD			0x80	/* SR write protect */

/* Define max times to check status register before we give up. */
#define	MAX_READY_WAIT_JIFFIES	(40 * HZ)	/* M25P16 specs 40s max chip erase */
#define	MAX_CMD_SIZE		5

#ifdef CONFIG_M25PXX_USE_FAST_READ
#define OPCODE_READ 	OPCODE_FAST_READ
#define FAST_READ_DUMMY_BYTE 1
#else
#define OPCODE_READ 	OPCODE_NORM_READ
#define FAST_READ_DUMMY_BYTE 0
#endif

#define JEDEC_MFR(_jedec_id)	((_jedec_id) >> 16)

/****************************************************************************/

struct flash_info {
	/* JEDEC id zero means "no ID" (most older chips); otherwise it has
	 * a high byte of zero plus three data bytes: the manufacturer id,
	 * then a two byte device id.
	 */
	u32		jedec_id;
	u16     ext_id;

	/* The size listed here is what works with OPCODE_SE, which isn't
	 * necessarily called a "sector" by the vendor.
	 */
	unsigned	sector_size;
	u16		n_sectors;

	u16		page_size;
	u16		addr_width;

	u16		flags;
#define	SECT_4K		0x01		/* OPCODE_BE_4K works uniformly */
#define	M25P_NO_ERASE	0x02		/* No erase command needed */
};

struct m25p {
	char *		name;
	dev_t			devt;
	spinlock_t		spi_lock;
	struct list_head	device_entry;
	struct spi_device	*spi;

	struct mutex		lock;
	unsigned		partitioned:1;
	u16			page_size;
	u16			addr_width;
	u8			erase_opcode;
	u8			*command;
	struct mutex		buf_lock;
	unsigned		users;
	u8			*buffer;

	struct gpio_flash_platform_data  *gpio_data;
	struct gpio_pin *gpios;


	size_t size;
	size_t erasesize;
	u32 flags;
	struct device dev;
	size_t numeraseregions;
	size_t *eraseregions;
	struct flash_info		*info;

};
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned bufsiz = 4096;//setcor size 4K=0xfff
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

#define GPIO_DONE	0x00 //in
#define GPIO_CONF	0x01 //out

#define GPIO_CS1	0x02 //out

/**
* @brief control the done_pin,config_pin and ce_pin status
* @param gpio:0(done_pin),1(config_pin),2(ce_pin),and specific to view the board-am335xevm.c
*        value:0(output low level),1(output high level)
* @return 0
* @note done_pin is the input pin
*/
int flash_spi_set_gpio(struct m25p *flash_spi,unsigned char gpio,unsigned char value)
{
	struct gpio_flash_platform_data  *gpio_func;
	if(gpio>2)
		return -1;
	gpio_func=flash_spi->gpio_data;
	gpio_func->gpio_set_val(flash_spi->gpios[gpio].gpio,value);

	return 0;
}

/**
* @brief get the done_pin,config_pin and ce_pin status
* @param gpio:0(done_pin),1(config_pin),2(ce_pin),and specific to view the board-am335xevm.c
* @return the gpio_pin status
*/
int flash_spi_get_gpio_val(struct m25p *flash_spi,unsigned char gpio)
{
	struct gpio_flash_platform_data  *gpio_func;
	if(gpio>2)
		return -1;
	gpio_func=flash_spi->gpio_data;
	return gpio_func->gpio_get_val(flash_spi->gpios[gpio].gpio);

}

/**
* @brief set the system work mode
* @param mode:GPIO_FLASH_SPI or GPIO_FLASH_IO_INPUT
* @return 0
*/
int flash_spi_gpio_mod_set(struct m25p *flash_spi,unsigned char mode)
{
	struct gpio_flash_platform_data  *gpio_func;
	gpio_func=flash_spi->gpio_data;
	return gpio_func->gpio_mod_set(gpio_func,mode);

}

static void flash_spi_complete(void *arg)
{
	complete(arg);
}
/**
* @brief Wrapper the spi_async (adding the param check,spin irq lock,wait completion)
* @return message status
* @note it is called by others spi communication function
*/
static ssize_t
flash_spi_sync(struct m25p *flash_spi, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = flash_spi_complete;
	message->context = &done;

	spin_lock_irq(&flash_spi->spi_lock);
	if (flash_spi->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(flash_spi->spi, message);
	spin_unlock_irq(&flash_spi->spi_lock);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
		if (status == 0)
			status = message->actual_length;
	}
	return status;
}

/**
* @brief flash_spi device info save the private data and assign the bufsize size 's buffer for flash_spi buffer
* @return 0:sucess
* @note mutex fuction and users counter funtion
	    file_operations
*/
static int
flash_spi_open(struct inode *inode, struct file *filp)
{
	struct m25p	*flash_spi;
	int			status = -ENXIO;

	mutex_lock(&device_list_lock);

	list_for_each_entry(flash_spi, &device_list, device_entry) {
		if (flash_spi->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status == 0) {
		if (!flash_spi->buffer) {
			flash_spi->buffer = kmalloc(bufsiz, GFP_KERNEL);
			if (!flash_spi->buffer) {
				dev_dbg(&flash_spi->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		if (status == 0) {
			flash_spi->users++;
			filp->private_data = flash_spi;
			//nonseekable_open(inode, filp);
		}
	} else
		pr_debug("flash_spi: nothing for minor %d\n", iminor(inode));

	mutex_unlock(&device_list_lock);
	return status;
}

/**
* @brief open function  anti-operating
* @return 0
* @note file_operations
*/
static int
flash_spi_release(struct inode *inode, struct file *filp)
{
	struct m25p	*flash_spi;
	int			status = 0;

	mutex_lock(&device_list_lock);
	flash_spi = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	flash_spi->users--;
	if (!flash_spi->users) {
		int		dofree;

		kfree(flash_spi->buffer);
		flash_spi->buffer = NULL;

		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&flash_spi->spi_lock);
		dofree = (flash_spi->spi == NULL);
		spin_unlock_irq(&flash_spi->spi_lock);

		if (dofree)
			kfree(flash_spi);
	}
	mutex_unlock(&device_list_lock);

	return status;
}



/****************************************************************************/

/*
 * Internal helper functions
 */

/*
 * Read the status register, returning its value in the location
 * Return the status register value.
 * Returns negative if error occurred.
 */
static int read_sr(struct m25p *flash)
{
	ssize_t retval;
	u8 code = OPCODE_RDSR;
	u8 val;

	retval = spi_write_then_read(flash->spi, &code, 1, &val, 1);

	if (retval < 0) {
		dev_err(&flash->spi->dev, "error %d reading SR\n",
				(int) retval);
		return retval;
	}

	return val;
}

/*
 * Write status register 1 byte
 * Returns negative if error occurred.
 */
static int write_sr(struct m25p *flash, u8 val)
{
	flash->command[0] = OPCODE_WRSR;
	flash->command[1] = val;

	return spi_write(flash->spi, flash->command, 2);
}

/*
 * Set write enable latch with Write Enable command.
 * Returns negative if error occurred.
 */
static inline int write_enable(struct m25p *flash)
{
	u8	code = OPCODE_WREN;

	return spi_write_then_read(flash->spi, &code, 1, NULL, 0);
}

/*
 * Send write disble instruction to the chip.
 */
static inline int write_disable(struct m25p *flash)
{
	u8	code = OPCODE_WRDI;

	return spi_write_then_read(flash->spi, &code, 1, NULL, 0);
}

/*
 * Enable/disable 4-byte addressing mode.
 */
static inline int set_4byte(struct m25p *flash, u32 jedec_id, int enable)
{
	switch (JEDEC_MFR(jedec_id)) {
// 	case CFI_MFR_MACRONIX:
// 		flash->command[0] = enable ? OPCODE_EN4B : OPCODE_EX4B;
// 		return spi_write(flash->spi, flash->command, 1);
	default:
		/* Spansion style */
		flash->command[0] = OPCODE_BRWR;
		flash->command[1] = enable << 7;
		return spi_write(flash->spi, flash->command, 2);
	}
}

/*
 * Service routine to read status register until ready, or timeout occurs.
 * Returns non-zero if error.
 */
static int wait_till_ready(struct m25p *flash)
{
	unsigned long deadline;
	int sr;

	deadline = jiffies + MAX_READY_WAIT_JIFFIES;

	do {
		if ((sr = read_sr(flash)) < 0)
			break;
		else if (!(sr & SR_WIP))
			return 0;

		cond_resched();

	} while (!time_after_eq(jiffies, deadline));

	return 1;
}

/**
* @brief Erase the whole flash memory
* @return 0 if successful, non-zero otherwise.
*/
static int
erase_chip(struct m25p *flash)
{
	printk("Erase the whole flash memory in the erase_chip\r\n");

	/* Wait until finished previous write command. */
	if (wait_till_ready(flash))
		return 1;

	/* Send write enable, then erase commands. */
	write_enable(flash);

	/* Set up command buffer. */
	flash->command[0] = OPCODE_CHIP_ERASE;

	spi_write(flash->spi, flash->command, 1);

	return 0;
}

/**
* @brief address adding into the send command
* @note opecode + address1+address2+address3...
*/
static void
m25p_addr2cmd(struct m25p *flash, unsigned int addr, u8 *cmd)
{
	/* opcode is in cmd[0] */
	cmd[1] = addr >> (flash->addr_width * 8 -  8);
	cmd[2] = addr >> (flash->addr_width * 8 - 16);
	cmd[3] = addr >> (flash->addr_width * 8 - 24);
	cmd[4] = addr >> (flash->addr_width * 8 - 32);
}

/**
* @brief computing the send command length
* @note 1(opecode)+addr length
* @return
*/
static int
m25p_cmdsz(struct m25p *flash)
{
	return 1 + flash->addr_width;//addr_width==4
}


/**
* @brief Erase one sector of flash memory at offset
        which is any address within the sector which should be erased.
* @return 0 if successful, non-zero otherwise.
*/
int
erase_sector(struct m25p *flash, u32 offset)
{
    printk("\n");
	/* Wait until finished previous write command. */
	if (wait_till_ready(flash))
		return 1;

	/* Send write enable, then erase commands. */
	write_enable(flash);

	/* Set up command buffer. */
	flash->command[0] = flash->erase_opcode;
	m25p_addr2cmd(flash, offset, flash->command);

	spi_write(flash->spi, flash->command, m25p_cmdsz(flash));

	return 0;
}

/****************************************************************************/
/*
 * MTD implementation
*/

/**
* @brief erase an address range on the flash chip.  The address range may extend
*        one or more erase sectors.
* @return Return an error there is a problem erasing.
* @note this fuction call the erase_chip and erase_sector to erase flash,and it is called flash_spi_ioctl and
*/

int flash_spi_erase(struct m25p *flash, struct erase_flash_info *instr)
{
	u32 addr,len;
	uint32_t rem;


	/* sanity checks */
	if (instr->addr + instr->len > flash->size)
		return -EINVAL;
	div_u64_rem(instr->len, flash->erasesize, &rem);
	if (rem)
		return -EINVAL;

	addr = instr->addr;
	len = instr->len;

	mutex_lock(&flash->lock);

	/* whole-chip erase? */
	if (len == flash->size) {
		if (erase_chip(flash)) {
			instr->state = MTD_ERASE_FAILED;
			mutex_unlock(&flash->lock);
			return -EIO;
		}

	} else {
		while (len) {
			if (erase_sector(flash, addr)) {
				instr->state = MTD_ERASE_FAILED;
				mutex_unlock(&flash->lock);
				printk("err:erase\r\n");
				return -EIO;
			}

			addr += flash->erasesize;
			len -= flash->erasesize;
		}
	}

	mutex_unlock(&flash->lock);

	instr->state = MTD_ERASE_DONE;
	//mtd_erase_callback(instr);
	//if(instr->callback!=NULL)//未初始化要出错
	//	instr->callback(instr);//Internal error: Oops - undefined instruction: 0 [#1]

	printk("erase OK\r\n");

	return 0;
}

/**
* @brief Read an address range from the flash chip.  The address range
*        may be any size provided it is within the physical boundaries.

*/

static ssize_t
flash_spi_read(struct file *filp, char __user *buf,
		size_t len, loff_t *f_pos)
{
	struct m25p *flash = filp->private_data;
	struct spi_transfer t[2];
	struct spi_message m;
	size_t retlen;
	size_t		missing;

    printk("%s: %s %s 0x%08x, len %zd\n",dev_name(&flash->spi->dev), __func__, "from",(u32)*f_pos, len);

	/* sanity checks */
	if (!len)
		return 0;
	if(len>bufsiz)
		return -1;

	if (*f_pos + len > flash->size)
		return -EINVAL;

	spi_message_init(&m);
	memset(t, 0, (sizeof t));

	/* NOTE:
	 * OPCODE_FAST_READ (if available) is faster.
	 * Should add 1 byte DUMMY_BYTE.
	 */
	t[0].tx_buf = flash->command;
	t[0].len = m25p_cmdsz(flash) + FAST_READ_DUMMY_BYTE;
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = flash->buffer;
	t[1].len = len;
	spi_message_add_tail(&t[1], &m);

	/* Byte count starts at zero. */
	retlen = 0;

	mutex_lock(&flash->lock);

	/* Wait till previous write/erase is done. */
	if (wait_till_ready(flash)) {
		/* REVISIT status return?? */
		mutex_unlock(&flash->lock);
		return -1;
	}

	/* FIXME switch to OPCODE_FAST_READ.  It's required for higher
	 * clocks; and at this writing, every chip this driver handles
	 * supports that opcode.
	 */

	/* Set up the write data buffer. */
	flash->command[0] = OPCODE_READ;
	m25p_addr2cmd(flash, *f_pos, flash->command);

	//spi_sync(flash->spi, &m);
	flash_spi_sync(flash, &m);

	retlen = m.actual_length - m25p_cmdsz(flash) - FAST_READ_DUMMY_BYTE;


	mutex_unlock(&flash->lock);
	missing = copy_to_user((char*)buf, (char*)flash->buffer, retlen);
	if (missing == retlen)
	{
		printk("spi err:%d %s\r\n",__LINE__,__FUNCTION__);
		retlen = -EFAULT;
	}
	else
	retlen = retlen - missing;
	//printk("read ok\r\n");
	*f_pos=*f_pos+retlen;
    printk("flash_spi_read\n\n");
	return retlen;
}

/**
* @brief Wrapper the flash_spi_read
* @note file_operations flash_spi_fops read
*/
static ssize_t
flash_spi_read_p(struct file *filp, char __user *buf,
		size_t len, loff_t *f_pos)
{
	struct m25p *flash = filp->private_data;
	if(flash->size<=SPI_FLASH_SIZE)
	{
		printk("err:flash size %x %x\r\n",flash->size,SPI_FLASH_SIZE);
		return -EINVAL;
	}

	if (*f_pos + len > SPI_FLASH_SIZE)
	{
        printk("%d %s\r\n",__LINE__,__FUNCTION__);
        return -EINVAL;
	}


	return flash_spi_read(filp,buf,len,f_pos);
}


/**
* @brief Write an address range to the flash chip.  Data must be written in
*        FLASH_PAGESIZE chunks.  The address range may be any size provided
*        it is within the physical boundaries.
* @note
*/
static int
flash_spi_write(struct file *filp, const char __user *buf,
		size_t len, loff_t *f_pos)
{
	struct m25p *flash =  filp->private_data;

	u32 page_offset, page_size;
	struct spi_transfer t[2];
	struct spi_message m;
	size_t retlen;
	size_t missing;


    printk("%s: %s %s 0x%08x, len %zd\n",
            dev_name(&flash->spi->dev), __func__, "to",
            (u32) *f_pos, len);

	retlen = 0;

	/* sanity checks */
	if (!len)
		return(0);
	if(len>bufsiz)
		return -1;

	if ( *f_pos + len > flash->size)
		return -EINVAL;



	missing = copy_from_user(flash->buffer, buf, len);
	if (missing != 0) {
		printk("err:%d %s\r\n",__LINE__,__FUNCTION__);
		printk("copy_from_user retrun %d\r\n",missing);
		return -EFAULT;
	}

	spi_message_init(&m);
	memset(t, 0, (sizeof t));

	t[0].tx_buf = flash->command;
	t[0].len = m25p_cmdsz(flash);
	spi_message_add_tail(&t[0], &m);




	t[1].tx_buf = flash->buffer;
	spi_message_add_tail(&t[1], &m);

	mutex_lock(&flash->lock);

	/* Wait until finished previous write command. */
	if (wait_till_ready(flash)) {
		mutex_unlock(&flash->lock);
		return -1;
	}

	write_enable(flash);

	/* Set up the opcode in the write buffer. */
	flash->command[0] = OPCODE_PP;
	m25p_addr2cmd(flash,  *f_pos, flash->command);

	page_offset =  *f_pos & (flash->page_size - 1);

	/* do all the bytes fit onto one page? */
	if (page_offset + len <= flash->page_size) {
		t[1].len = len;

		//spi_sync(flash->spi, &m);
		flash_spi_sync(flash, &m);

		retlen = m.actual_length - m25p_cmdsz(flash);
	} else {
		u32 i;

		/* the size of data remaining on the first page */
		page_size = flash->page_size - page_offset;

		t[1].len = page_size;
		//spi_sync(flash->spi, &m);
		flash_spi_sync(flash, &m);

		retlen = m.actual_length - m25p_cmdsz(flash);

		/* write everything in flash->page_size chunks */
		for (i = page_size; i < len; i += page_size) {
			page_size = len - i;
			if (page_size > flash->page_size)
				page_size = flash->page_size;

			/* write the next page to flash */
			m25p_addr2cmd(flash,  *f_pos + i, flash->command);

			t[1].tx_buf = flash->buffer + i;
			t[1].len = page_size;

			wait_till_ready(flash);

			write_enable(flash);

			//spi_sync(flash->spi, &m);
			flash_spi_sync(flash, &m);

			retlen += m.actual_length - m25p_cmdsz(flash);
		}
	}

	mutex_unlock(&flash->lock);
	*f_pos=*f_pos+retlen;

	printk("flash_spi_write\n\n");
	return retlen;
}

/**
* @brief Wrapper the flash_spi_write
* @note file_operations flash_spi_fops write
*/
static ssize_t
flash_spi_write_p(struct file *filp, const char __user *buf,
		size_t len, loff_t *f_pos)
{

	struct m25p *flash = filp->private_data;
	if(flash->size<=SPI_FLASH_SIZE)
	{
		printk("err:flash size %x %x\r\n",flash->size,SPI_FLASH_SIZE);
		return -EINVAL;
	}

	if (*f_pos + len > SPI_FLASH_SIZE)
		return -EINVAL;

	return flash_spi_write(filp,buf,len,f_pos);
}

/**
* @brief Wrapper the flash_spi_write
* @note file_operations flash_spi_fops write
*/


static inline int unsigned_offsets(struct file *file)
{
	return file->f_mode & FMODE_UNSIGNED_OFFSET;
}

/**
* @brief
*
*
*/
loff_t
flash_spi_llseek(struct file *file, loff_t offset, int origin)
{
	struct inode *inode = file->f_path.dentry->d_inode;
	loff_t retval;
	loff_t new_offset=0;

	//printk(KERN_ERR"offset:%x,origin%d\r\n",(int)offset,origin);
	mutex_lock(&inode->i_mutex);
	switch (origin) {
		case SEEK_SET:
			new_offset=offset;
			break;
		case SEEK_END:
			new_offset=offset + i_size_read(inode);
			break;
		case SEEK_CUR:
			if (offset == 0) {
				retval = file->f_pos;
				goto out;
			}
			new_offset=offset +file->f_pos;
			break;
		case SEEK_DATA:
			/*
			 * In the generic case the entire file is data, so as
			 * long as offset isn't at the end of the file then the
			 * offset is data.
			 */
			if (offset >= inode->i_size) {
				retval = -ENXIO;
				goto out;
			}
			break;
		case SEEK_HOLE:
			/*
			 * There is a virtual hole at the end of the file, so
			 * as long as offset isn't i_size or larger, return
			 * i_size.
			 */
			if (offset >= inode->i_size) {
				retval = -ENXIO;
				goto out;
			}
			new_offset = inode->i_size;
			break;
	}
	retval = -EINVAL;
	if (new_offset >= 0 || unsigned_offsets(file)) {
		if (new_offset != file->f_pos) {
			file->f_pos = new_offset;
			file->f_version = 0;
		}
		retval = new_offset;
	}
out:
	mutex_unlock(&inode->i_mutex);
	return retval;
}

/**
*
*
*
*
*
*
*/

static long
flash_spi_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int			err = 0;
	int			retval = 0;
	struct m25p	*flash_spi;
	struct spi_device	*spi;
	u32			tmp;
//	unsigned		n_ioc;
//	struct spi_ioc_transfer	*ioc;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	flash_spi = filp->private_data;
	spin_lock_irq(&flash_spi->spi_lock);
	spi = spi_dev_get(flash_spi->spi);
	spin_unlock_irq(&flash_spi->spi_lock);

	if (spi == NULL)
		return -ESHUTDOWN;

	/* use the buffer lock here for triple duty:
	 *  - prevent I/O (from us) so calling spi_setup() is safe;
	 *  - prevent concurrent SPI_IOC_WR_* from morphing
	 *    data fields while SPI_IOC_RD_* reads them;
	 *  - SPI_IOC_MESSAGE needs the buffer locked "normally".
	 */
	mutex_lock(&flash_spi->buf_lock);

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
	case FLASH_SPI_CONF:
	{
		retval = __get_user(tmp, (__u32 __user *)arg);
		if (retval == 0) {
				flash_spi_set_gpio(flash_spi,GPIO_CONF,tmp?1:0);
		}

	}break;
	case FLASH_SPI_DONE:
	{
		tmp=flash_spi_get_gpio_val(flash_spi,GPIO_DONE);
		retval = __put_user(tmp, (__u32 __user *)arg);
	}break;

	case FLASH_SPI_CS1:
	{
		retval = __get_user(tmp, (__u32 __user *)arg);
		if (retval == 0) {

				flash_spi_set_gpio(flash_spi,GPIO_CS1,tmp?1:0);

		}
	}break;

	case FLASH_SPI_ERASE:
	{
 		struct erase_flash_info instr;

 		instr.addr=0;
 		instr.len=SPI_FLASH_SIZE;
		instr.callback=NULL;
		flash_spi_erase(flash_spi,&instr);
	}break;
	case FLASH_SN_ERASE:
	{
 		struct erase_flash_info instr;

 		instr.addr=SPI_FLASH_SIZE;
 		instr.len=SPI_SN_SIZE;
		instr.callback=NULL;
		flash_spi_erase(flash_spi,&instr);

	}break;
	case FLASH_SN_READ:
	{
		//flash_sn_t *sn=(flash_sn_t*)arg;

		//flash_spi_read(flash_spi,)
		flash_spi_llseek(filp, SPI_FLASH_SIZE, SEEK_SET);
		retval=flash_spi_read(filp,(void __user *)arg,SPI_SN_SIZE,&filp->f_pos);
		if(retval>=0)retval=0;
	}break;
	case FLASH_SN_WRITE:
	{
		struct erase_flash_info instr;

 		instr.addr=SPI_FLASH_SIZE;
 		instr.len=SPI_SN_SIZE;
		instr.callback=NULL;
		flash_spi_erase(flash_spi,&instr);
		flash_spi_llseek(filp, SPI_FLASH_SIZE, SEEK_SET);
		if (JEDEC_MFR(flash_spi->info->jedec_id) == CFI_MFR_SST)//flash->write = sst_write;
			;//retval=flash_spi_sst_write(filp,(void __user *)arg,SPI_SN_SIZE,&filp->f_pos);
		else
			retval=flash_spi_write(filp,(void __user *)arg,SPI_SN_SIZE,&filp->f_pos);

		if(retval>=0)retval=0;

	}break;
	case FLASH_GPIO_MODE:
	{
		retval = __get_user(tmp, (__u32 __user *)arg);
		if (retval == 0) {
			flash_spi_gpio_mod_set(flash_spi,tmp?1:0);
		}
	}break;


	}

	mutex_unlock(&flash_spi->buf_lock);
	spi_dev_put(spi);
	return retval;
}

#ifdef CONFIG_COMPAT
static long
flash_spi_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return flash_spi_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define flash_spi_compat_ioctl NULL
#endif /* CONFIG_COMPAT */



/****************************************************************************/

/*
 * SPI device driver setup and teardown
 */



#define INFO(_jedec_id, _ext_id, _sector_size, _n_sectors, _flags)	\
	((kernel_ulong_t)&(struct flash_info) {				\
		.jedec_id = (_jedec_id),				\
		.ext_id = (_ext_id),					\
		.sector_size = (_sector_size),				\
		.n_sectors = (_n_sectors),				\
		.page_size = 256,					\
		.flags = (_flags),					\
	})

#define CAT25_INFO(_sector_size, _n_sectors, _page_size, _addr_width)	\
	((kernel_ulong_t)&(struct flash_info) {				\
		.sector_size = (_sector_size),				\
		.n_sectors = (_n_sectors),				\
		.page_size = (_page_size),				\
		.addr_width = (_addr_width),				\
		.flags = M25P_NO_ERASE,					\
	})

/* NOTE: double check command sets and memory organization when you add
 * more flash chips.  This current list focusses on newer chips, which
 * have been converging on command sets which including JEDEC ID.
 */
static const struct spi_device_id m25p_ids[] = {
	/* Atmel -- some are (confusingly) marketed as "DataFlash" */
	{ "at25fs010",  INFO(0x1f6601, 0, 32 * 1024,   4, SECT_4K) },
	{ "at25fs040",  INFO(0x1f6604, 0, 64 * 1024,   8, SECT_4K) },

	{ "at25df041a", INFO(0x1f4401, 0, 64 * 1024,   8, SECT_4K) },
	{ "at25df641",  INFO(0x1f4800, 0, 64 * 1024, 128, SECT_4K) },

	{ "at26f004",   INFO(0x1f0400, 0, 64 * 1024,  8, SECT_4K) },
	{ "at26df081a", INFO(0x1f4501, 0, 64 * 1024, 16, SECT_4K) },
	{ "at26df161a", INFO(0x1f4601, 0, 64 * 1024, 32, SECT_4K) },
	{ "at26df321",  INFO(0x1f4700, 0, 64 * 1024, 64, SECT_4K) },

	/* EON -- en25xxx */
	{ "en25f32", INFO(0x1c3116, 0, 64 * 1024,  64, SECT_4K) },
	{ "en25p32", INFO(0x1c2016, 0, 64 * 1024,  64, 0) },
	{ "en25p64", INFO(0x1c2017, 0, 64 * 1024, 128, 0) },

	/* Intel/Numonyx -- xxxs33b */
	{ "160s33b",  INFO(0x898911, 0, 64 * 1024,  32, 0) },
	{ "320s33b",  INFO(0x898912, 0, 64 * 1024,  64, 0) },
	{ "640s33b",  INFO(0x898913, 0, 64 * 1024, 128, 0) },

	/* Macronix */
	{ "mx25l4005a",  INFO(0xc22013, 0, 64 * 1024,   8, SECT_4K) },
	{ "mx25l8005",   INFO(0xc22014, 0, 64 * 1024,  16, 0) },
	{ "mx25l1606e",  INFO(0xc22015, 0, 64 * 1024,  32, SECT_4K) },
	{ "mx25l3205d",  INFO(0xc22016, 0, 64 * 1024,  64, 0) },
	{ "mx25l6405d",  INFO(0xc22017, 0, 64 * 1024, 128, 0) },
	{ "mx25l12805d", INFO(0xc22018, 0, 64 * 1024, 256, 0) },
	{ "mx25l12855e", INFO(0xc22618, 0, 64 * 1024, 256, 0) },
	{ "mx25l25635e", INFO(0xc22019, 0, 64 * 1024, 512, 0) },
	{ "mx25l25655e", INFO(0xc22619, 0, 64 * 1024, 512, 0) },

	/* Spansion -- single (large) sector size only, at least
	 * for the chips listed here (without boot sectors).
	 */
	{ "s25sl004a",  INFO(0x010212,      0,  64 * 1024,   8, 0) },
	{ "s25sl008a",  INFO(0x010213,      0,  64 * 1024,  16, 0) },
	{ "s25sl016a",  INFO(0x010214,      0,  64 * 1024,  32, 0) },
	{ "s25sl032a",  INFO(0x010215,      0,  64 * 1024,  64, 0) },
	{ "s25sl032p",  INFO(0x010215, 0x4d00,  64 * 1024,  64, SECT_4K) },
	{ "s25sl064a",  INFO(0x010216,      0,  64 * 1024, 128, 0) },
	{ "s25fl256s0", INFO(0x010219, 0x4d00, 256 * 1024, 128, 0) },
	{ "s25fl256s1", INFO(0x010219, 0x4d01,  64 * 1024, 512, 0) },
	{ "s25fl512s",  INFO(0x010220, 0x4d00, 256 * 1024, 256, 0) },
	{ "s70fl01gs",  INFO(0x010221, 0x4d00, 256 * 1024, 256, 0) },
	{ "s25sl12800", INFO(0x012018, 0x0300, 256 * 1024,  64, 0) },
	{ "s25sl12801", INFO(0x012018, 0x0301,  64 * 1024, 256, 0) },
	{ "s25fl129p0", INFO(0x012018, 0x4d00, 256 * 1024,  64, 0) },
	{ "s25fl129p1", INFO(0x012018, 0x4d01,  64 * 1024, 256, 0) },
	{ "s25fl016k",  INFO(0xef4015,      0,  64 * 1024,  32, SECT_4K) },
	{ "s25fl064k",  INFO(0xef4017,      0,  64 * 1024, 128, SECT_4K) },

	/* SST -- large erase sizes are "overlays", "sectors" are 4K */
	{ "sst25vf040b", INFO(0xbf258d, 0, 64 * 1024,  8, SECT_4K) },
	{ "sst25vf080b", INFO(0xbf258e, 0, 64 * 1024, 16, SECT_4K) },
	{ "sst25vf016b", INFO(0xbf2541, 0, 64 * 1024, 32, SECT_4K) },
	{ "sst25vf032b", INFO(0xbf254a, 0, 64 * 1024, 64, SECT_4K) },
	{ "sst25wf512",  INFO(0xbf2501, 0, 64 * 1024,  1, SECT_4K) },
	{ "sst25wf010",  INFO(0xbf2502, 0, 64 * 1024,  2, SECT_4K) },
	{ "sst25wf020",  INFO(0xbf2503, 0, 64 * 1024,  4, SECT_4K) },
	{ "sst25wf040",  INFO(0xbf2504, 0, 64 * 1024,  8, SECT_4K) },

	/* ST Microelectronics -- newer production may have feature updates */
	{ "m25p05",  INFO(0x202010,  0,  32 * 1024,   2, 0) },
	{ "m25p10",  INFO(0x202011,  0,  32 * 1024,   4, 0) },
	{ "m25p20",  INFO(0x202012,  0,  64 * 1024,   4, 0) },
	{ "m25p40",  INFO(0x202013,  0,  64 * 1024,   8, 0) },
	{ "flash_spi",  INFO(0x202014,  0,  64 * 1024,  16, 0) },
	{ "m25p16",  INFO(0x202015,  0,  64 * 1024,  32, 0) },
	{ "m25p32",  INFO(0x202016,  0,  64 * 1024,  64, 0) },
	{ "m25p64",  INFO(0x202017,  0,  64 * 1024, 128, 0) },
	{ "m25p128", INFO(0x202018,  0, 256 * 1024,  64, 0) },

	{ "m25p05-nonjedec",  INFO(0, 0,  32 * 1024,   2, 0) },
	{ "m25p10-nonjedec",  INFO(0, 0,  32 * 1024,   4, 0) },
	{ "m25p20-nonjedec",  INFO(0, 0,  64 * 1024,   4, 0) },
	{ "m25p40-nonjedec",  INFO(0, 0,  64 * 1024,   8, 0) },
	{ "flash_spi-nonjedec",  INFO(0, 0,  64 * 1024,  16, 0) },
	{ "m25p16-nonjedec",  INFO(0, 0,  64 * 1024,  32, 0) },
	{ "m25p32-nonjedec",  INFO(0, 0,  64 * 1024,  64, 0) },
	{ "m25p64-nonjedec",  INFO(0, 0,  64 * 1024, 128, 0) },
	{ "m25p128-nonjedec", INFO(0, 0, 256 * 1024,  64, 0) },

	{ "m45pe10", INFO(0x204011,  0, 64 * 1024,    2, 0) },
	{ "m45pe80", INFO(0x204014,  0, 64 * 1024,   16, 0) },
	{ "m45pe16", INFO(0x204015,  0, 64 * 1024,   32, 0) },

	{ "m25pe80", INFO(0x208014,  0, 64 * 1024, 16,       0) },
	{ "m25pe16", INFO(0x208015,  0, 64 * 1024, 32, SECT_4K) },

	{ "m25px32",    INFO(0x207116,  0, 64 * 1024, 64, SECT_4K) },
	{ "m25px32-s0", INFO(0x207316,  0, 64 * 1024, 64, SECT_4K) },
	{ "m25px32-s1", INFO(0x206316,  0, 64 * 1024, 64, SECT_4K) },
	{ "m25px64",    INFO(0x207117,  0, 64 * 1024, 128, 0) },

	/* Winbond -- w25x "blocks" are 64K, "sectors" are 4KiB */
	{ "w25x10", INFO(0xef3011, 0, 64 * 1024,  2,  SECT_4K) },
	{ "w25x20", INFO(0xef3012, 0, 64 * 1024,  4,  SECT_4K) },
	{ "w25x40", INFO(0xef3013, 0, 64 * 1024,  8,  SECT_4K) },
	{ "w25x80", INFO(0xef3014, 0, 64 * 1024,  16, SECT_4K) },
	{ "w25x16", INFO(0xef3015, 0, 64 * 1024,  32, SECT_4K) },
	{ "w25x32", INFO(0xef3016, 0, 64 * 1024,  64, SECT_4K) },
	{ "w25q32", INFO(0xef4016, 0, 64 * 1024,  64, SECT_4K) },
	{ "w25x64", INFO(0xef3017, 0, 64 * 1024, 128, SECT_4K) },
	{ "w25q64", INFO(0xef4017, 0, 64 * 1024, 128, SECT_4K) },

	/* Catalyst / On Semiconductor -- non-JEDEC */
	{ "cat25c11", CAT25_INFO(  16, 8, 16, 1) },
	{ "cat25c03", CAT25_INFO(  32, 8, 16, 2) },
	{ "cat25c09", CAT25_INFO( 128, 8, 32, 2) },
	{ "cat25c17", CAT25_INFO( 256, 8, 32, 2) },
	{ "cat25128", CAT25_INFO(2048, 8, 64, 2) },
	{ },
};
MODULE_DEVICE_TABLE(spi, m25p_ids);

const struct spi_device_id *__devinit jedec_probe(struct spi_device *spi)
{
	int			tmp;
	u8			code = OPCODE_RDID;
	u8			id[5];
	u32			jedec;
	u16                     ext_jedec;
	struct flash_info	*info;

	/* JEDEC also defines an optional "extended device information"
	 * string for after vendor-specific data, after the three bytes
	 * we use here.  Supporting some chips might require using it.
	 */
	tmp = spi_write_then_read(spi, &code, 1, id, 5);
	if (tmp < 0) {
// 		DEBUG(MTD_DEBUG_LEVEL0, "%s: error %d reading JEDEC ID\n",
// 			dev_name(&spi->dev), tmp);
		return ERR_PTR(tmp);
	}
	jedec = id[0];
	jedec = jedec << 8;
	jedec |= id[1];
	jedec = jedec << 8;
	jedec |= id[2];

    printk("***********Manufacturer ID :0x%0x******************\n",jedec&0x0F);
    printk("***********Memory Type     :0x%0x******************\n",jedec&0x0FF0);
    printk("***********Capacity ID     :0x%0x *****************\n",jedec&0x0FF000);


	ext_jedec = id[3] << 8 | id[4];

	for (tmp = 0; tmp < ARRAY_SIZE(m25p_ids) - 1; tmp++) {
		info = (void *)m25p_ids[tmp].driver_data;
		if (info->jedec_id == jedec) {
			if (info->ext_id != 0 && info->ext_id != ext_jedec)
				continue;
			//printk("m25p_ids 2:%d\r\n",tmp);
			return &m25p_ids[tmp];
		}
	}
	dev_err(&spi->dev, "unrecognized JEDEC id %06x\n", jedec);
	return ERR_PTR(-ENODEV);
}


static struct file_operations flash_spi_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.read  =	flash_spi_read_p,
	.write =	flash_spi_write_p,

	.unlocked_ioctl = flash_spi_ioctl,
	.compat_ioctl = flash_spi_compat_ioctl,
	.open =		flash_spi_open,
	.release =	flash_spi_release,
	.llseek =	flash_spi_llseek,
};


/*
 * board specific setup should have ensured the SPI clock used here
 * matches what the READ command supports, at least until this driver
 * understands FAST_READ (for clocks over 25 MHz).
 */
static int __devinit flash_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id	*id = spi_get_device_id(spi);
	struct gpio_flash_platform_data	*data;
	struct m25p			*flash;
	int			status;
	struct flash_info		*info;
	unsigned			i;
//	unsigned			cs_pin;
	//struct mtd_partition		*parts = NULL;
//	int				nr_parts = 0;
	unsigned long		minor;

	/* Platform data helps sort out which chip type we have, as
	 * well as how this board partitions it.  If we don't have
	 * a chip ID, try the JEDEC id commands; they'll work for most
	 * newer chips, even if we don't recognize the particular chip.
	 */

	data = spi->dev.platform_data;//control done config ce
	if (data && data->type) {
		const struct spi_device_id *plat_id;

		for (i = 0; i < ARRAY_SIZE(m25p_ids) - 1; i++) {
			plat_id = &m25p_ids[i];
			if (strcmp(data->type, plat_id->name))
				continue;
				printk("m25p_ids:%d\r\n",i);
			break;
		}

		if (i < ARRAY_SIZE(m25p_ids) - 1)
			id = plat_id;
		else
			dev_warn(&spi->dev, "unrecognized id %s\n", data->type);
	}

	info = (void *)id->driver_data;
    printk("***********jedec_id   :0x%0x******************\n",info->jedec_id);
    printk("***********ext_id     :0x%0x******************\n",info->ext_id);

    printk("***********sector_size:0x%0x *****************\n",info->sector_size);
    printk("***********n_sectors  :0x%0x******************\n",info->n_sectors);
    printk("***********page_size  :0x%0x *****************\n",info->page_size);
    printk("***********addr_width :0x%0x******************\n",info->addr_width);
    printk("***********flags      :0x%0x *****************\n",info->flags);


	flash = kzalloc(sizeof *flash, GFP_KERNEL);
	if (!flash)
		return -ENOMEM;
	flash->command = kmalloc(MAX_CMD_SIZE + FAST_READ_DUMMY_BYTE, GFP_KERNEL);
	if (!flash->command) {
		kfree(flash);
		return -ENOMEM;
	}

	flash->spi = spi;
	mutex_init(&flash->lock);
	dev_set_drvdata(&spi->dev, flash);
	spin_lock_init(&flash->spi_lock);
	mutex_init(&flash->buf_lock);
	INIT_LIST_HEAD(&flash->device_entry);
	flash->gpio_data=spi->dev.platform_data;
	flash->gpios=(struct gpio_flash *)flash->gpio_data->gpios;

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		flash->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(flash_spi_class, &spi->dev, flash->devt,
				    flash, "flash_spi%d.%d",
				    spi->master->bus_num, spi->chip_select);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&flash->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	if (status == 0)
		spi_set_drvdata(spi, flash);
	else
		kfree(flash);


    if(JEDEC_MFR(info->jedec_id) == CFI_MFR_WINBOND)
    {
        write_enable(flash);
        wait_till_ready(flash);
        jedec_probe(flash->spi);
    }

	if (data && data->name)
		flash->name = (char*)data->name;//w25q32
	else
		flash->name = (char*)dev_name(&spi->dev);

	//flash->type = MTD_NORFLASH;
	//flash->writesize = 1;
	flash->flags = MTD_CAP_NORFLASH;

	flash->size = info->sector_size * info->n_sectors;
	printk("info size:%x,sector_size:%x,n_sectors:%d\r\n",flash->size,info->sector_size,info->n_sectors);
	//flash->erase = flash_spi_erase;
	//flash->read = flash_spi_read;

	/* sst flash chips use AAI word program */
	if (JEDEC_MFR(info->jedec_id) == CFI_MFR_SST)//flash->write = sst_write;
	{
        printk("flash_spi_fops.write=flash_spi_sst_write_p\n");
        //flash_spi_fops.write=flash_spi_sst_write_p;
	}

	else
	{
        printk("flash_spi_fops.write=flash_spi_write_p\n");
        flash_spi_fops.write = flash_spi_write_p;
	}


	/* prefer "small sector" erase if possible */
	if (info->flags & SECT_4K)
	{
		printk("flash->erase_opcode = OPCODE_BE_4K\n");
		flash->erase_opcode = OPCODE_BE_4K;
		flash->erasesize = SPI_SN_SIZE;
	} else
	{
        printk("flash->erase_opcode = OPCODE_SE\n");
		flash->erase_opcode = OPCODE_SE;
		flash->erasesize = info->sector_size;
	}

	if (info->flags & M25P_NO_ERASE)
		flash->flags |= MTD_NO_ERASE;

	flash->dev.parent = &spi->dev;
	flash->page_size = info->page_size;

	if (info->addr_width)
		flash->addr_width = info->addr_width;
	else {
        //enable 4-byte addressing if the device exceeds 16MiB
        flash->addr_width = 3;
	}
	flash->info=info;

	printk("%s (%lld Kbytes)\n", id->name,
			(long long)flash->size >> 10);

	printk("mtd .name = %s, .size = 0x%llx (%lldMiB) "
			".erasesize = 0x%.8x (%uKiB) .numeraseregions = %d\n",
		flash->name,
		(long long)flash->size, (long long)(flash->size >> 20),
		flash->erasesize, flash->erasesize / 1024,
		flash->numeraseregions);

	return 0;
}

static int __devexit flash_spi_remove(struct spi_device *spi)
{
	struct m25p	*flash_spi = spi_get_drvdata(spi);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&flash_spi->spi_lock);
	flash_spi->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&flash_spi->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&flash_spi->device_entry);
	device_destroy(flash_spi_class, flash_spi->devt);
	clear_bit(MINOR(flash_spi->devt), minors);
	if (flash_spi->users == 0)
	{
		//printk("err:%s,%d\r\n",__FUNCTION__,__LINE__);
		kfree(flash_spi->command);
		//kfree(flash_spi);
	}
	mutex_unlock(&device_list_lock);

	return 0;
}

static struct spi_driver flash_spi_driver = {
	.driver = {
		.name	= "flash_spi",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.id_table	= m25p_ids,
	.probe	= flash_spi_probe,
	.remove	= __devexit_p(flash_spi_remove),

	/* REVISIT: many of these chips have deep power-down modes, which
	 * should clearly be entered on suspend() to minimize power use.
	 * And also when they're otherwise idle...
	 */
};

/*-------------------------------------------------------------------------*/

static int __init flash_spi_init(void)
{
	int status;

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, "flash_spi", &flash_spi_fops);
	if (status < 0)
		return status;

	flash_spi_class = class_create(THIS_MODULE, "flash_spi");
	if (IS_ERR(flash_spi_class)) {
		unregister_chrdev(SPIDEV_MAJOR, flash_spi_driver.driver.name);
		return PTR_ERR(flash_spi_class);
	}

	status = spi_register_driver(&flash_spi_driver);
	if (status < 0) {
		class_destroy(flash_spi_class);
		unregister_chrdev(SPIDEV_MAJOR, flash_spi_driver.driver.name);
	}
	return status;
}
module_init(flash_spi_init);

static void __exit flash_spi_exit(void)
{
	spi_unregister_driver(&flash_spi_driver);
	class_destroy(flash_spi_class);
	unregister_chrdev(SPIDEV_MAJOR, flash_spi_driver.driver.name);
}
module_exit(flash_spi_exit);

MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Jone Chen <chenhuayun@mooncell.com.cn>");
//MODULE_ALIAS("spi:flash_spi");
