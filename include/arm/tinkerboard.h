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

#define MRAA_TINKERBOARD_GPIO_COUNT 28
#define MRAA_TINKERBOARD_I2C_COUNT  3
#define MRAA_TINKERBOARD_SPI_COUNT  2
#define MRAA_TINKERBOARD_UART_COUNT 4
#define MRAA_TINKERBOARD_PWM_COUNT  2
#define MRAA_TINKERBOARD_AIO_COUNT  1
#define MRAA_TINKERBOARD_PIN_COUNT  40

mraa_board_t *
        mraa_tinkerboard();

#ifdef __cplusplus
}
#endif
