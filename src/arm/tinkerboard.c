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

#include "arm/tinkerboard.h"
#include "common.h"

#define DT_BASE "/proc/device-tree"
/* 
* "Radxa ROCK Pi 4" is the model name on stock 5.x kernels
* "ROCK PI 4A", "ROCK PI 4B" is used on Radxa 4.4 kernel
* so we search for the string below by ignoring case
*/
#define PLATFORM_NAME_TINKERBOARD "Rockchip RK3288 Asus Tinker Board"
#define MAX_SIZE 64

const char* tinkerboard_serialdev[MRAA_TINKERBOARD_UART_COUNT] = { "/dev/ttyS0", "/dev/ttyS1", "/dev/ttyS2", "/dev/ttyS3" };

void
mraa_tinkerboard_pininfo(mraa_board_t* board, int index, int sysfs_pin, mraa_pincapabilities_t pincapabilities_t, char* fmt, ...)
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

mraa_board_t*
mraa_tinkerboard()
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
    b->phy_pin_count = MRAA_TINKERBOARD_PIN_COUNT + 1;

    if (mraa_file_exist(DT_BASE "/model")) {
        // We are on a modern kernel, great!!!!
        if (mraa_file_contains(DT_BASE "/model", PLATFORM_NAME_TINKERBOARD)) {
            b->platform_name = PLATFORM_NAME_TINKERBOARD;
            b->uart_dev[0].device_path = (char*) tinkerboard_serialdev[0];
            b->uart_dev[1].device_path = (char*) tinkerboard_serialdev[1];
            b->uart_dev[2].device_path = (char*) tinkerboard_serialdev[2];
            b->uart_dev[3].device_path = (char*) tinkerboard_serialdev[3];
        }
    }

    // UART
    b->uart_dev_count = MRAA_TINKERBOARD_UART_COUNT;
    b->def_uart_dev = 1;
   // b->uart_dev[0].index = 0;
   // b->uart_dev[1].index = 1;

    // I2C
    if (strncmp(b->platform_name, PLATFORM_NAME_TINKERBOARD, MAX_SIZE) == 0) {
        b->i2c_bus_count = MRAA_TINKERBOARD_I2C_COUNT;
        b->def_i2c_bus = 0;
        b->i2c_bus[0].bus_id = 7;
        b->i2c_bus[1].bus_id = 2;
        b->i2c_bus[2].bus_id = 6;
    }

    // SPI
    b->spi_bus_count = MRAA_TINKERBOARD_SPI_COUNT;
    b->def_spi_bus = 0;
    b->spi_bus[0].bus_id = 32766;
    b->spi_bus[1].bus_id = 32765;

    b->pwm_dev_count = MRAA_TINKERBOARD_PWM_COUNT;
    b->pwm_default_period = 500;
    b->pwm_max_period = 2147483;
    b->pwm_min_period = 1;

    b->pins = (mraa_pininfo_t*) malloc(sizeof(mraa_pininfo_t) * b->phy_pin_count);
    if (b->pins == NULL) {
        free(b->adv_func);
        free(b);
        return NULL;
    }

    b->pins[32].pwm.parent_id = 1;
    b->pins[33].pwm.parent_id = 0;

    b->aio_count = MRAA_TINKERBOARD_AIO_COUNT;
    b->adc_raw = 10;
    b->adc_supported = 10;
    b->aio_dev[0].pin = 12;
    b->aio_non_seq = 1;
                                          /* valid, gpio, pwm, fast_gpio, spi, i2c, aio, uart */
    mraa_tinkerboard_pininfo(b, 0,   -1, (mraa_pincapabilities_t){0,0,0,0,0,0,0,0}, "INVALID");
    mraa_tinkerboard_pininfo(b, 1,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "VCC3.3V_IO");
    mraa_tinkerboard_pininfo(b, 2,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "VCC5V_SYS");
    mraa_tinkerboard_pininfo(b, 3,  252, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "GP8A4_I2C1_SDA");
    mraa_tinkerboard_pininfo(b, 4,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "VCC5V_SYS");
    mraa_tinkerboard_pininfo(b, 5,  253, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "GP8A5_I2C1_SCL");
    mraa_tinkerboard_pininfo(b, 6,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_tinkerboard_pininfo(b, 7,   17, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "GP0C1_CLKOUT");
    mraa_tinkerboard_pininfo(b, 8,  161, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "GP5B1_UART1TX");
    mraa_tinkerboard_pininfo(b, 9,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_tinkerboard_pininfo(b, 10, 160, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "GP5B0_UART1RX");
    mraa_tinkerboard_pininfo(b, 11, 164, (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "GP5B4_SPI0CLK_UART4CTSN");
    mraa_tinkerboard_pininfo(b, 12, 184, (mraa_pincapabilities_t){1,1,0,0,0,1,1,0}, "GP6A0_PCM/I2S_CLK");
    mraa_tinkerboard_pininfo(b, 13, 166, (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "GP5B6_SPI0_TXD_UART4TX");
    mraa_tinkerboard_pininfo(b, 14,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_tinkerboard_pininfo(b, 15, 167, (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "GP5B7_SPI0_RXD_UART4RX");
    mraa_tinkerboard_pininfo(b, 16, 162, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GP5B2_UART1CTSN");
    mraa_tinkerboard_pininfo(b, 17,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "VCC33_IO");
    mraa_tinkerboard_pininfo(b, 18, 163, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "GP5B3_UART1RTSN");
    mraa_tinkerboard_pininfo(b, 19, 257, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "GP8B1_SPI2TXD");
    mraa_tinkerboard_pininfo(b, 20,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_tinkerboard_pininfo(b, 21, 256, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "GP8B0_SPI2RXD");
    mraa_tinkerboard_pininfo(b, 22, 171, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GP5C3");
    mraa_tinkerboard_pininfo(b, 23, 254, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "GP8A6_SPI2CLK");
    mraa_tinkerboard_pininfo(b, 24, 255, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "GP8A7_SPI2CSN0");
    mraa_tinkerboard_pininfo(b, 25,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_tinkerboard_pininfo(b, 26, 251, (mraa_pincapabilities_t){1,0,0,0,1,0,0,0}, "GP8A3_SPI2CSN1");
    mraa_tinkerboard_pininfo(b, 27, 233, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "GP7C1_I2C4_SDA");
    mraa_tinkerboard_pininfo(b, 28, 234, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "GP7C2_I2C4_SCL");
    mraa_tinkerboard_pininfo(b, 29, 165, (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "GP5B5_SPI0CSN0_UART4RTSN");
    mraa_tinkerboard_pininfo(b, 30,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_tinkerboard_pininfo(b, 31, 168, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "GP5C0_SPI0CSN1");
    mraa_tinkerboard_pininfo(b, 32, 239, (mraa_pincapabilities_t){1,1,1,0,0,0,0,1}, "GP7C7_UART2TX_PWM3");
    mraa_tinkerboard_pininfo(b, 33, 238, (mraa_pincapabilities_t){1,1,1,0,0,0,0,1}, "GP7C6_UART2RX_PWM2");
    mraa_tinkerboard_pininfo(b, 34,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_tinkerboard_pininfo(b, 35, 185, (mraa_pincapabilities_t){1,1,0,0,0,1,1,0}, "GP6A1_PCM/I2S_FS");
    mraa_tinkerboard_pininfo(b, 36, 223, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "GP7A7_UART3RX");
    mraa_tinkerboard_pininfo(b, 37, 224, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "GP7B0_UART3TX");
    mraa_tinkerboard_pininfo(b, 38, 187, (mraa_pincapabilities_t){1,1,0,0,0,1,1,0}, "GP6A3_PCM/I2S_SDI");
    mraa_tinkerboard_pininfo(b, 39,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_tinkerboard_pininfo(b, 40, 188, (mraa_pincapabilities_t){1,1,0,0,0,1,1,0}, "GP6A4_PCM/I2S_SDO");

    return b;
}
