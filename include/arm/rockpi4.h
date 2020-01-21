/*
 * Author: Brian <brian@vamrs.com>
 * Copyright (c) 2019 Vamrs Corporation.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "mraa_internal.h"

#define MRAA_ROCKPI4_GPIO_COUNT 27
#define MRAA_ROCKPI4_I2C_COUNT  3
#define MRAA_ROCKPI4_SPI_COUNT  2
#define MRAA_ROCKPI4_UART_COUNT 2
#define MRAA_ROCKPI4_PWM_COUNT  2
#define MRAA_ROCKPI4_AIO_COUNT  1
#define MRAA_ROCKPI4_PIN_COUNT  40
#define MRAA_ROCKPI4_GRF_REGISTER  				0xFF770000
#define MRAA_ROCKPI4_GRF_REGISTER_OFFSET		0x0004
#define MRAA_ROCKPI4_GRF_REGISTER_OFFSET_START  0x0E000
#define MRAA_ROCKPI4_PMU_REGISTER  				0xFF320000
#define MRAA_ROCKPI4_PMU_REGISTER_OFFSET		0x0004
#define MRAA_ROCKPI4_PMU_REGISTER_OFFSET_START	0x00000
#define MRAA_ROCKPI4_CLOCKGATE					0xFF760000
/*
 * CRU_CLKGATE_CON31
 */
#define MRAA_ROCKPI4_CLOCKGATE_OFFSET			0x037C
#define MRAA_ROCKPI4_SWPORTA_DR_OFFSET			0x0000
#define MRAA_ROCKPI4_SWPORTA_DDR_OFFSET			0x0004
#define MRAA_ROCKPI4_EXT_PORTA_OFFSET			0x0050
#define MRAA_ROCKPI4_GPIO_GROUP_PIN_COUNT		8
#define MRAA_ROCKPI4_GPIO_BANK_PIN_COUNT		32
#define MRAA_ROCKPI4_INTERNAL_PIN_COUNT		    160
#define MRAA_ROCKPI4_WRITE_ENABLE_BIT_OFFSET	16
#define MRAA_ROCKPI4_IOMUX_GPIO_MASK			0x03
#define MRAA_ROCKPI4_CLOCK_MASK					0x01
#define MRAA_ROCKPI4_GPIO_DIRECTION_IN			0x00
#define MRAA_ROCKPI4_GPIO_DIRECTION_OUT			0x01
#define MRAA_ROCKPI4_GPIO_ENABLE_MASK			0x01
#define MRAA_ROCKPI4_GPIO_DISABLE_MASK			0x00

static const char MRAA_ROCKPI4_GPIO_GROUPS[] 			  = {'A', 'B', 'C', 'D'};
static const uint8_t MRAA_ROCKPI4_GPIO_GROUP_COUNT	      = ((uint8_t)sizeof(MRAA_ROCKPI4_GPIO_GROUPS) / sizeof(MRAA_ROCKPI4_GPIO_GROUPS[0]));


typedef enum {
	MRAA_ROCKCHIP_REG_FILE_INVALID = -1,
	MRAA_ROCKCHIP_GRF = 0,
	MRAA_ROCKCHIP_PMU = 1
} mraa_rockchip_register_file_t;

typedef enum {
	MRAA_ROCKCHIP_CLOCK_ENABLED = 0,
	MRAA_ROCKCHIP_CLOCK_DISABLED = 1
} mraa_rockchip_clock_state_t;

typedef struct {
	uint8_t group;
	const char* name;
	uint32_t iomux_address;
} mraa_rockchip_groupinfo_t;

typedef struct {
	uint8_t bank;
	const uint32_t* address;
	mraa_rockchip_groupinfo_t groups[MRAA_ROCKPI4_GPIO_GROUP_COUNT];
	mraa_rockchip_register_file_t register_file;
	mraa_rockchip_clock_state_t default_clock_state;
	mraa_rockchip_clock_state_t clock_state;
} mraa_rockchip_bankinfo_t;

typedef struct {
	uint8_t pin;
	mraa_rockchip_bankinfo_t* bankinfo;
	mraa_rockchip_groupinfo_t* groupinfo;
} mraa_rockchip_pininfo_t;

mraa_board_t *
        mraa_rockpi4();


#ifdef __cplusplus
}
#endif
