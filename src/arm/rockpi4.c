/*
 * Author: Brian <brian@vamrs.com>
 * Copyright (c) 2019 Vamrs Corporation.
 *
 * SPDX-License-Identifier: MIT
 */

#include <mraa/common.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>

#include "arm/rockpi4.h"
#include "common.h"

#define DT_BASE "/proc/device-tree"
/* 
* "Radxa ROCK Pi 4" is the model name on stock 5.x kernels
* "ROCK PI 4A", "ROCK PI 4B" is used on Radxa 4.4 kernel
* so we search for the string below by ignoring case
*/
#define PLATFORM_NAME_ROCK_PI_4 "ROCK Pi 4"
#define PLATFORM_NAME_ROCK_PI_4A "ROCK PI 4A"
#define PLATFORM_NAME_ROCK_PI_4B "ROCK PI 4B"
#define MAX_SIZE 64
#define MMAP_PATH "/dev/mem"

const char* rockpi4_serialdev[MRAA_ROCKPI4_UART_COUNT] = { "/dev/ttyS2","/dev/ttyS4"};
// MMAP
static uint32_t* mmap_reg[MRAA_ROCKPI4_GPIO_BANK_COUNT] = {NULL};
static uint32_t* mmap_reg_iomux = NULL;
static uint32_t* mmap_reg_clock = NULL;
static int mmap_fd[MRAA_ROCKPI4_GPIO_BANK_COUNT] = {NULL};
static int mmap_fd_iomux = 0;
static int mmap_fd_clock = 0;
static int mmap_size = 4096UL;
static mraa_rockchip_pininfo_t mraa_rockpi4_pinmap[MRAA_ROCKPI4_PIN_COUNT] = {NULL};
static mraa_rockchip_pininfo_t* pins[MRAA_ROCKPI4_INTERNAL_PIN_COUNT] = {NULL};
static mraa_rockchip_bankinfo_t* banks[MRAA_ROCKPI4_GPIO_BANK_COUNT] = {NULL};
static mraa_rockchip_bankinfo_t* groups[MRAA_ROCKPI4_GPIO_GROUP_COUNT] = {NULL};
static unsigned int mmap_count_bank[MRAA_ROCKPI4_GPIO_BANK_COUNT] = {0};
static unsigned int mmap_count = 0;

void
mraa_rockpi4_pininfo(mraa_board_t* board, int index, int sysfs_pin, mraa_pincapabilities_t pincapabilities_t, char* fmt, ...)
{
    va_list arg_ptr;
    if (index > board->phy_pin_count)
        return;

    mraa_pininfo_t* pininfo = &board->pins[index];
    va_start(arg_ptr, fmt);
    vsnprintf(pininfo->name, MRAA_PIN_NAME_SIZE, fmt, arg_ptr);

    if( pincapabilities_t.gpio == 1 ) {
	va_arg(arg_ptr, int);
	pininfo->gpio.gpio_chip = va_arg(arg_ptr, int);
	pininfo->gpio.gpio_line = va_arg(arg_ptr, int);
    }

    pininfo->capabilities = pincapabilities_t;

    va_end(arg_ptr);
    pininfo->gpio.pinmap = sysfs_pin;
    pininfo->gpio.mux_total = 0;
}

static mraa_result_t
mraa_rockpi4_mmap_unsetup(int bank)
{
    if (mmap_reg[bank] == NULL) {
        syslog(LOG_WARNING, "rockpi4 mmap: null register nothing to unsetup");
        return MRAA_ERROR_INVALID_RESOURCE;
    }
    munmap(mmap_reg[bank], mmap_size);
    mmap_reg[bank] = NULL;
    close(mmap_fd[bank]);
    if (mmap_count <= 0) {
    	mraa_result_t result = mraa_rockchip_unmap_clock();
    	if (result != MRAA_SUCCESS) {
    		return result;
    	}
    }
    return MRAA_SUCCESS;
}

mraa_result_t
mraa_rockpi4_mmap_write(mraa_gpio_context dev, int value)
{
    uint32_t address = *mraa_rockpi4_pinmap[dev->phy_pin]->bankinfo->address;
    uint8_t  bank = mraa_rockpi4_pinmap[dev->phy_pin]->bankinfo->bank;
    uint8_t  group = mraa_rockpi4_pinmap[dev->phy_pin]->group;
    uint32_t register_value_direction;
    uint32_t register_value;
    register_value_direction = *(volatile uint32_t*) (mmap_reg[bank] + MRAA_ROCKPI4_SWPORTA_DDR_OFFSET);
    if (address == NULL || bank == NULL || group == NULL) {
    	syslog(LOG_ERR, "rockpi4 mmap: pin not initialized for mmap write");
    	return MRAA_ERROR_INVALID_RESOURCE;
    }
    register_value_direction = *(volatile uint32_t*) (mmap_reg[bank] + MRAA_ROCKPI4_SWPORTA_DDR_OFFSET);
    register_value = *(volatile uint32_t*) (mmap_reg[bank] + MRAA_ROCKPI4_SWPORTA_DR_OFFSET);
    mraa_rockchip_set_gpio_value(&register_value_direction, MRAA_ROCKPI4_GPIO_DIRECTION_OUT, group * MRAA_ROCKPI4_GPIO_GROUP_PIN_COUNT + dev->pin);
    mraa_rockchip_set_gpio_value(&register_value_direction, value, group * MRAA_ROCKPI4_GPIO_GROUP_PIN_COUNT + dev->pin);

    return MRAA_SUCCESS;
}

int
mraa_rockpi4_mmap_read(mraa_gpio_context dev)
{
    uint32_t value;
    uint32_t offset = (0x1000 * dev->pin);

    value = *(volatile uint32_t*) (mmap_reg + offset + 0x04);
    if (value & (uint32_t)(1 << 0)) {
        return 1;
    }

    return 0;
}

mraa_result_t
mraa_rockpi4_mmap_setup(mraa_gpio_context dev, mraa_boolean_t enable)
{
    if (dev == NULL) {
        syslog(LOG_ERR, "rockpi4 mmap: context not valid");
        return MRAA_ERROR_INVALID_HANDLE;
    }
	uint8_t iomux_bit = 0;
	uint8_t iomux_write_enable = 0;
    mraa_rockchip_pininfo_t* pininfo = mraa_rockchip_setup(dev->pin);

    /* disable mmap if already enabled */
    if (enable == 0) {
        if (dev->mmap_write == NULL && dev->mmap_read == NULL) {
            syslog(LOG_ERR, "rockpi4 mmap: can't disable disabled mmap gpio");
            return MRAA_ERROR_INVALID_PARAMETER;
        }
        dev->mmap_write = NULL;
        dev->mmap_read = NULL;
        mmap_count_bank[pininfo->bankinfo->bank]--;
        mmap_count--;
        if (mmap_count_bank[pininfo->bankinfo->bank] == 0) {
            return mraa_rockpi4_mmap_unsetup(pininfo->bankinfo->bank);
        }
        return MRAA_SUCCESS;
    }

    if (dev->mmap_write != NULL && dev->mmap_read != NULL) {
        syslog(LOG_ERR, "rockpi4 mmap: can't enable enabled mmap gpio");
        return MRAA_ERROR_INVALID_PARAMETER;
    }

    if (mmap_reg[pininfo->bankinfo->bank] == NULL) {
        if ((mmap_fd[pininfo->bankinfo->bank] = open(MMAP_PATH, O_RDWR)) < 0) {
            syslog(LOG_ERR, "rockpi4 mmap: unable to open /dev/mem");
            return MRAA_ERROR_INVALID_HANDLE;
        }
        mmap_reg[pininfo->bankinfo->bank] = mmap(NULL, mmap_size, PROT_READ | PROT_WRITE, MAP_SHARED, mmap_fd[pininfo->bankinfo->bank], *pininfo->bankinfo->address);

        if (mmap_reg[pininfo->bankinfo->bank] == MAP_FAILED) {
            syslog(LOG_ERR, "rockpi4 mmap: failed to mmap");
            mmap_reg[pininfo->bankinfo->bank] = NULL;
            close(mmap_fd[pininfo->bankinfo->bank]);
            return MRAA_ERROR_NO_RESOURCES;
        }
    }

    if (*pininfo->bankinfo->register_file == MRAA_ROCKCHIP_GRF) {
        mraa_result_t result = 	mraa_rockchip_mmap_clock();
    	if (result != MRAA_SUCCESS) {
    			return result;
    	}
    	if (mraa_rockchip_is_clock_disabled(pininfo->bankinfo)) {
    		pininfo->bankinfo->default_clock_state = MRAA_ROCKCHIP_CLOCK_DISABLED;
    		mraa_rockchip_set_clock_state(pininfo->bankinfo, 1);
    		pininfo->bankinfo->clock_state = MRAA_ROCKCHIP_CLOCK_ENABLED;
    	}
    }

    if ((mmap_fd_iomux = open(MMAP_PATH, O_RDWR)) < 0) {
                syslog(LOG_ERR, "rockpi4 mmap: unable to open /dev/mem for IOMUX");
                return MRAA_ERROR_INVALID_HANDLE;
    }

    mmap_reg_iomux = mmap(NULL, mmap_size, PROT_READ | PROT_WRITE, MAP_SHARED, mmap_fd_iomux, pininfo->groupinfo->iomux_address);
    if (mmap_reg_iomux == MAP_FAILED) {
         syslog(LOG_ERR, "rockpi4 mmap: failed to mmap IOMUX");
         mmap_reg_iomux = NULL;
         close(mmap_fd_iomux);
         return MRAA_ERROR_NO_RESOURCES;
     }
    iomux_bit = dev->pin * 2;
    iomux_write_enable = iomux_bit + MRAA_ROCKPI4_WRITE_ENABLE_BIT_OFFSET;
    mraa_rockchip_register_active_low(&mmap_reg_iomux, MRAA_ROCKPI4_IOMUX_GPIO_MASK, iomux_bit, iomux_write_enable);
    munmap(mmap_reg_iomux, mmap_size);
    mmap_reg_iomux = NULL;
    close(mmap_fd_iomux);

    if (*pininfo->bankinfo->default_clock_state == MRAA_ROCKCHIP_CLOCK_DISABLED) {
    	mraa_rockchip_set_clock_state(pininfo->bankinfo, 0);
    	pininfo->bankinfo->clock_state = MRAA_ROCKCHIP_CLOCK_DISABLED;
    }

    mraa_rockchip_pininfo_t rockchip_pininfo = (mraa_rockchip_pininfo_t*) calloc(1, sizeof(mraa_rockchip_pininfo_t));
    mraa_rockpi4_pinmap[dev->phy_pin] = rockchip_pininfo;

    dev->mmap_write = &mraa_rockpi4_mmap_write;
    dev->mmap_read = &mraa_rockpi4_mmap_read;
    mmap_count_bank[pininfo->bankinfo->bank]++;
    mmap_count++;

    return MRAA_SUCCESS;
}

mraa_board_t*
mraa_rockpi4()
{
    mraa_board_t* b = (mraa_board_t*) calloc(1, sizeof(mraa_board_t));
    if (b == NULL) {
        return NULL;
    }

    b->adv_func = (mraa_adv_func_t*) calloc(1, sizeof(mraa_adv_func_t));
    if (b->adv_func == NULL) {
        free(b);
        return NULL;
    }

    // pin mux for buses are setup by default by kernel so tell mraa to ignore them
    b->no_bus_mux = 1;
    b->phy_pin_count = MRAA_ROCKPI4_PIN_COUNT + 1;

    if (mraa_file_exist(DT_BASE "/model")) {
        // We are on a modern kernel, great!!!!
        if (mraa_file_contains(DT_BASE "/model", PLATFORM_NAME_ROCK_PI_4)  ||
            mraa_file_contains(DT_BASE "/model", PLATFORM_NAME_ROCK_PI_4A) ||
            mraa_file_contains(DT_BASE "/model", PLATFORM_NAME_ROCK_PI_4B)
            ) {
            b->platform_name = PLATFORM_NAME_ROCK_PI_4;
            b->uart_dev[0].device_path = (char*) rockpi4_serialdev[0];
            b->uart_dev[1].device_path = (char*) rockpi4_serialdev[1];
        }
    }

    // UART
    b->uart_dev_count = MRAA_ROCKPI4_UART_COUNT;
    b->def_uart_dev = 0;
    b->uart_dev[0].index = 2;
    b->uart_dev[1].index = 4;

    // I2C
    if (strncmp(b->platform_name, PLATFORM_NAME_ROCK_PI_4, MAX_SIZE) == 0) {
        b->i2c_bus_count = MRAA_ROCKPI4_I2C_COUNT;
        b->def_i2c_bus = 0;
        b->i2c_bus[0].bus_id = 7;
        b->i2c_bus[1].bus_id = 2;
        b->i2c_bus[2].bus_id = 6;
    }

    // SPI
    b->spi_bus_count = MRAA_ROCKPI4_SPI_COUNT;
    b->def_spi_bus = 0;
    b->spi_bus[0].bus_id = 32766;
    b->spi_bus[1].bus_id = 32765;

    b->pwm_dev_count = MRAA_ROCKPI4_PWM_COUNT;
    b->pwm_default_period = 500;
    b->pwm_max_period = 2147483;
    b->pwm_min_period = 1;

    b->pins = (mraa_pininfo_t*) calloc(b->phy_pin_count, sizeof(mraa_pininfo_t));
    if (b->pins == NULL) {
        free(b->adv_func);
        free(b);
        return NULL;
    }

    b->pins[11].pwm.parent_id = 0;
    b->pins[11].pwm.mux_total = 0;
    b->pins[11].pwm.pinmap = 0;
    b->pins[13].pwm.parent_id = 1;
    b->pins[13].pwm.mux_total = 0;
    b->pins[13].pwm.pinmap = 0;

    b->aio_count = MRAA_ROCKPI4_AIO_COUNT;
    b->adc_raw = 10;
    b->adc_supported = 10;
    b->aio_dev[0].pin = 26;
    b->aio_non_seq = 1;

    mraa_rockpi4_pininfo(b, 0,   -1, (mraa_pincapabilities_t){0,0,0,0,0,0,0,0}, "INVALID");
    mraa_rockpi4_pininfo(b, 1,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "3V3");
    mraa_rockpi4_pininfo(b, 2,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "5V");
    mraa_rockpi4_pininfo(b, 3,   71, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "SDA7");
    mraa_rockpi4_pininfo(b, 4,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "5V");
    mraa_rockpi4_pininfo(b, 5,   72, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "SCL7");
    mraa_rockpi4_pininfo(b, 6,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_rockpi4_pininfo(b, 7,   75, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI2_CLK");
    mraa_rockpi4_pininfo(b, 8,  148, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "TXD2");
    mraa_rockpi4_pininfo(b, 9,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_rockpi4_pininfo(b, 10, 147, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "RXD2");
    mraa_rockpi4_pininfo(b, 11, 146, (mraa_pincapabilities_t){1,1,1,0,0,0,0,0}, "PWM0");
    mraa_rockpi4_pininfo(b, 12, 131, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO4_A3");
    mraa_rockpi4_pininfo(b, 13, 150, (mraa_pincapabilities_t){1,1,1,0,0,0,0,0}, "PWM1");
    mraa_rockpi4_pininfo(b, 14,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_rockpi4_pininfo(b, 15, 149, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO4_C5");
    mraa_rockpi4_pininfo(b, 16, 154, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO4_D2");
    mraa_rockpi4_pininfo(b, 17,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "3V3");
    mraa_rockpi4_pininfo(b, 18, 156, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO4_D4");
    mraa_rockpi4_pininfo(b, 19,  40, (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "SPI1TX,TXD4");
    mraa_rockpi4_pininfo(b, 20,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_rockpi4_pininfo(b, 21,  39, (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "SPI1RX,RXD4");
    mraa_rockpi4_pininfo(b, 22, 157, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO4_D5");
    mraa_rockpi4_pininfo(b, 23,  41, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI1CLK");
    mraa_rockpi4_pininfo(b, 24,  42, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI1CS");
    mraa_rockpi4_pininfo(b, 25,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_rockpi4_pininfo(b, 26,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,1,0}, "ADC_IN0");
    mraa_rockpi4_pininfo(b, 27,  64, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "SDA2");
    mraa_rockpi4_pininfo(b, 28,  65, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "SCL2");
    mraa_rockpi4_pininfo(b, 29,  74, (mraa_pincapabilities_t){1,1,0,0,1,1,0,0}, "SCL6,SPI2RX");
    mraa_rockpi4_pininfo(b, 30,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_rockpi4_pininfo(b, 31,  73, (mraa_pincapabilities_t){1,1,0,0,1,1,0,0}, "SDA6,SPI2TX");
    mraa_rockpi4_pininfo(b, 32, 112, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO3_C0");
    mraa_rockpi4_pininfo(b, 33,  76, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI2CS");
    mraa_rockpi4_pininfo(b, 34,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_rockpi4_pininfo(b, 35, 133, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO4_A5");
    mraa_rockpi4_pininfo(b, 36, 132, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO4_A4");
    mraa_rockpi4_pininfo(b, 37, 158, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO4_D6");
    mraa_rockpi4_pininfo(b, 38, 134, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO4_A6");
    mraa_rockpi4_pininfo(b, 39,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_rockpi4_pininfo(b, 40, 135, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO4_A7");

    return b;
}
static mraa_rockchip_pininfo_t* mraa_rockchip_setup(uint8_t pin) {
		uint8_t bank;
		uint8_t group;
		mraa_rockchip_pininfo_t* pininfo;
		if (pins[pin] == NULL) {
			pins[pin] = (mraa_rockchip_pininfo_t*) calloc(1, sizeof(mraa_rockchip_pininfo_t));
		} else {
			return pins[pin];
		}
		bank = pin / MRAA_ROCKPI4_GPIO_BANK_PIN_COUNT;
		group = (pin - bank * MRAA_ROCKPI4_GPIO_BANK_PIN_COUNT) / MRAA_ROCKPI4_GPIO_GROUP_PIN_COUNT;
		if (banks[bank] == NULL) {
			banks[bank] =  (mraa_rockchip_bankinfo_t*) calloc(1, sizeof(mraa_rockchip_bankinfo_t));
			mraa_rockchip_setup_bank(banks[bank], bank);
		}
		pins[pin]->bankinfo = banks[bank];
		pins[pin]->pin = (pin - bank * MRAA_ROCKPI4_GPIO_BANK_PIN_COUNT) % MRAA_ROCKPI4_GPIO_GROUP_PIN_COUNT;
		pins[pin]->groupinfo = banks[bank]->groups[group];
		return pins[pin];
}
static mraa_result_t mraa_rockchip_setup_bank(mraa_rockchip_bankinfo_t* bankinfo, uint8_t bank) {
	free(bankinfo);
	bankinfo =  (mraa_rockchip_bankinfo_t*) calloc(1, sizeof(mraa_rockchip_bankinfo_t));
	bankinfo->bank = bank;
	bankinfo->address = &MRAA_ROCKPI4_GPIO_BANK_ADDRESSES[bank];
	for (int i = 0; i < MRAA_ROCKPI4_GRF_BANK_COUNT; i++) {
	    	if (MRAA_ROCKPI4_GPIO_GRF_BANKS[i] == bank) {
	    		bankinfo->register_file = MRAA_ROCKCHIP_GRF;
	    	}
	    }
	    if (bankinfo->register_file == NULL) {
	    	for(int i = 0; i < MRAA_ROCKPI4_PMU_BANK_COUNT; i++) {
	    		if (MRAA_ROCKPI4_GPIO_PMU_BANKS[i] == bank) {
	    			bankinfo->register_file = MRAA_ROCKCHIP_PMU;
	    		    }
	    	}
	    }
	    if (bankinfo->register_file == NULL) {
	    	free(bankinfo);
	    	syslog(LOG_ERR, "rockpi4 mmap: bank is invalid");
	    	return MRAA_ERROR_INVALID_HANDLE;
	    }

	 for (int i = 0; i < MRAA_ROCKPI4_GPIO_GROUP_COUNT; i++) {
		 mraa_rockchip_groupinfo_t* groupinfo[i];
		 mraa_rockchip_setup_group(bankinfo, groupinfo[i], i);
	 }
	 return MRAA_SUCCESS;
}
static mraa_result_t mraa_rockchip_setup_group(mraa_rockchip_bankinfo_t* bankinfo, mraa_rockchip_groupinfo_t* groupinfo, uint8_t group) {
	free(groupinfo);
	groupinfo = (mraa_rockchip_groupinfo_t*) calloc(1, sizeof(mraa_rockchip_groupinfo_t));
	groupinfo->group = group;
	if (bankinfo->register_file == MRAA_ROCKCHIP_PMU) {
		groupinfo->iomux_address = MRAA_ROCKPI4_PMU_REGISTER + MRAA_ROCKPI4_PMU_REGISTER_OFFSET_START + bankinfo->bank * 16 + group * MRAA_ROCKPI4_PMU_REGISTER_OFFSET;
	} else if (bankinfo->register_file == MRAA_ROCKCHIP_GRF) {
		groupinfo->iomux_address = MRAA_ROCKPI4_GRF_REGISTER + MRAA_ROCKPI4_GRF_REGISTER_OFFSET_START + (bankinfo->bank - MRAA_ROCKPI4_PMU_BANK_COUNT) * 16 + group * MRAA_ROCKPI4_GRF_REGISTER_OFFSET;

	} else {
		free(groupinfo);
		return MRAA_ERROR_INVALID_HANDLE;
	}
	groupinfo->name = &MRAA_ROCKPI4_GPIO_GROUPS[group];
	return MRAA_SUCCESS;
}
static int mraa_rockchip_register_get_bit(uint32_t register_value, int offset) {
	return ((register_value) >> (offset)) & 1;
}
static void mraa_rockchip_register_active_low(volatile uint32_t* register_value, uint8_t bits, uint32_t enable_bit, uint32_t write_enable_bit) {
	int enable_mask = 0;
	int write_enable_mask = 0;
	enable_mask = bits<<enable_bit;
	enable_mask = ~enable_mask;
	write_enable_mask = bits<<write_enable_bit;
	*register_value &= enable_mask;
	*register_value |= write_enable_mask;
}
static void mraa_rockchip_register_active_high(volatile uint32_t* register_value, uint8_t bits, uint32_t enable_bit, uint32_t write_enable_bit) {
	int enable_mask = 0;
	int write_enable_mask = 0;
	enable_mask = bits<<enable_bit;
	write_enable_mask = bits<<write_enable_bit;
	*register_value |= enable_mask;
	*register_value |= write_enable_mask;
}
static void mraa_rockchip_set_gpio_value(volatile uint32_t* register_value, uint8_t bits, uint32_t enable_bit) {
	int enable_mask = 0;
	if (bits) {
		enable_mask = bits<<enable_bit;
		*register_value |= enable_mask;
	} else {
		enable_mask = 1<<enable_bit;
		enable_mask = ~enable_mask;
		*register_value &= enable_mask;
	}
}
static mraa_result_t mraa_rockchip_mmap_clock() {
	if (mmap_reg_clock == NULL) {
		if ((mmap_fd_clock = open(MMAP_PATH, O_RDWR)) < 0) {
			syslog(LOG_ERR, "rockpi4 mmap: unable to open /dev/mem for CLOCK ENABLE");
			return MRAA_ERROR_INVALID_HANDLE;
		}
		mmap_reg_clock = mmap(NULL, mmap_size, PROT_READ | PROT_WRITE, MAP_SHARED, mmap_fd_clock, (MRAA_ROCKPI4_CLOCKGATE + MRAA_ROCKPI4_CLOCKGATE_OFFSET));
		if (mmap_reg_clock == MAP_FAILED) {
			syslog(LOG_ERR, "rockpi4 mmap: failed to mmap CLOCK ENABLE");
			mmap_reg_clock = NULL;
			close(mmap_fd_clock);
			return MRAA_ERROR_NO_RESOURCES;
		}
	}
	return MRAA_SUCCESS;
}
static mraa_result_t mraa_rockchip_unmap_clock() {
	if (mmap_reg_clock != NULL) {
		munmap(mmap_reg_clock, mmap_size);
		mmap_reg_clock = NULL;
		close(mmap_fd_clock);
	}
	return MRAA_SUCCESS;
}
static void mraa_rockchip_set_clock_state(mraa_rockchip_bankinfo_t* bankinfo, mraa_boolean_t enable) {
	if (enable) {
		mraa_rockchip_register_active_low(&mmap_reg_clock, MRAA_ROCKPI4_CLOCK_MASK, (bankinfo->bank + 1), (bankinfo->bank + 1 + MRAA_ROCKPI4_WRITE_ENABLE_BIT_OFFSET));
	} else {
		mraa_rockchip_register_active_high(&mmap_reg_clock, MRAA_ROCKPI4_CLOCK_MASK, (bankinfo->bank + 1), (bankinfo->bank + 1 + MRAA_ROCKPI4_WRITE_ENABLE_BIT_OFFSET));
	}

}
static mraa_boolean_t mraa_rockchip_is_clock_disabled(mraa_rockchip_bankinfo_t* bankinfo) {
	uint32_t register_value = *(volatile uint32_t*) (mmap_reg_clock);
	return mraa_rockchip_register_get_bit(register_value, bankinfo->bank + 1);
}

