#ifndef __LINUX_GPIO_FLASH_H
#define __LINUX_GPIO_FLASH_H

#define MTD_WRITEABLE		0x400	/* Device is writeable */
#define MTD_BIT_WRITEABLE	0x800	/* Single bits can be flipped */
#define MTD_NO_ERASE		0x1000	/* No erase necessary */
#define MTD_POWERUP_LOCK	0x2000	/* Always locked after reset */

/* Some common devices / combinations of capabilities */
#define MTD_CAP_ROM		0
#define MTD_CAP_RAM		(MTD_WRITEABLE | MTD_BIT_WRITEABLE | MTD_NO_ERASE)
#define MTD_CAP_NORFLASH	(MTD_WRITEABLE | MTD_BIT_WRITEABLE)
#define MTD_CAP_NANDFLASH	(MTD_WRITEABLE)

#define MTD_CHAR_MAJOR 90
#define MTD_BLOCK_MAJOR 31

#define MTD_ERASE_PENDING	0x01
#define MTD_ERASING		0x02
#define MTD_ERASE_SUSPEND	0x04
#define MTD_ERASE_DONE		0x08
#define MTD_ERASE_FAILED	0x10

#define MTD_FAIL_ADDR_UNKNOWN -1LL

#define CFI_MFR_ANY		0xFFFF
#define CFI_ID_ANY		0xFFFF
#define CFI_MFR_CONTINUATION	0x007F

#define CFI_MFR_AMD		0x0001
#define CFI_MFR_AMIC		0x0037
#define CFI_MFR_ATMEL		0x001F
#define CFI_MFR_EON		0x001C
#define CFI_MFR_FUJITSU		0x0004
#define CFI_MFR_HYUNDAI		0x00AD
#define CFI_MFR_INTEL		0x0089
#define CFI_MFR_MACRONIX	0x00C2
#define CFI_MFR_NEC		0x0010
#define CFI_MFR_PMC		0x009D
#define CFI_MFR_SAMSUNG		0x00EC
#define CFI_MFR_SHARP		0x00B0
#define CFI_MFR_SST		0x00BF
#define CFI_MFR_ST		0x0020 /* STMicroelectronics */
#define CFI_MFR_TOSHIBA		0x0098
#define CFI_MFR_WINBOND		0x00DA



struct gpio_pin {
	const char *name;
	unsigned 	gpio;
};

#define GPIO_FLASH_SPI		0x01
#define GPIO_FLASH_IO_INPUT	0x00

struct gpio_flash_platform_data {
	const char *type;
	const char *name;
	int 	num_gpios;
	char 	mode;
	const struct gpio_pin *gpios;
	int		(*gpio_set_val)(unsigned gpio,char val);
	int		(*gpio_get_val)(unsigned gpio);
	int 	(*gpio_mod_set)(struct gpio_flash_platform_data *gpio_data,int mod);
};

struct gpio_spi_bus_platform_data {
	const char *type;
	const char *name;
	int 	num_gpios;
	char 	mode;
	const struct gpio_pin *gpios;
	int		(*gpio_set_val)(unsigned gpio,char val);
	int		(*gpio_get_val)(unsigned gpio);
	int 	(*gpio_mod_set)(struct gpio_spi_bus_platform_data *gpio_data,int mod);
};


struct erase_flash_info {
	uint64_t addr;
	uint64_t len;
	uint64_t fail_addr;
	u_long time;
	u_long retries;
	unsigned dev;
	unsigned cell;
	void (*callback) (struct erase_flash_info *self);
	u_long priv;
	u_char state;
};

#endif

