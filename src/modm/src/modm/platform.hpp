/*
 * Copyright (c) 2016, Sascha Schade
 * Copyright (c) 2017, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_PLATFORM_HPP
#define MODM_PLATFORM_HPP

#include <modm/architecture/utils.hpp>

#include "platform/clock/common.hpp"
#include "platform/clock/rcc.hpp"
#include "platform/clock/systick_timer.hpp"
#include "platform/core/atomic_lock.hpp"
#include "platform/core/delay.hpp"
#include "platform/core/flash_reader.hpp"
#include "platform/core/hardware_init.hpp"
#include "platform/core/heap_table.hpp"
#include "platform/core/peripherals.hpp"
#include "platform/gpio/base.hpp"
#include "platform/gpio/connector.hpp"
#include "platform/gpio/connector_detail.hpp"
#include "platform/gpio/gpio_A0.hpp"
#include "platform/gpio/gpio_A1.hpp"
#include "platform/gpio/gpio_A10.hpp"
#include "platform/gpio/gpio_A11.hpp"
#include "platform/gpio/gpio_A12.hpp"
#include "platform/gpio/gpio_A13.hpp"
#include "platform/gpio/gpio_A14.hpp"
#include "platform/gpio/gpio_A15.hpp"
#include "platform/gpio/gpio_A2.hpp"
#include "platform/gpio/gpio_A3.hpp"
#include "platform/gpio/gpio_A4.hpp"
#include "platform/gpio/gpio_A5.hpp"
#include "platform/gpio/gpio_A6.hpp"
#include "platform/gpio/gpio_A7.hpp"
#include "platform/gpio/gpio_A8.hpp"
#include "platform/gpio/gpio_A9.hpp"
#include "platform/gpio/gpio_B0.hpp"
#include "platform/gpio/gpio_B1.hpp"
#include "platform/gpio/gpio_B3.hpp"
#include "platform/gpio/gpio_B4.hpp"
#include "platform/gpio/gpio_B5.hpp"
#include "platform/gpio/gpio_B6.hpp"
#include "platform/gpio/gpio_B7.hpp"
#include "platform/gpio/gpio_C14.hpp"
#include "platform/gpio/gpio_C15.hpp"
#include "platform/gpio/gpio_H3.hpp"
#include "platform/gpio/inverted.hpp"
#include "platform/gpio/port.hpp"
#include "platform/gpio/set.hpp"
#include "platform/gpio/software_port.hpp"
#include "platform/gpio/unused.hpp"
#include "platform/i2c/i2c_master_1.hpp"
#include "platform/i2c/i2c_timing_calculator.hpp"
#include "platform/spi/spi_base.hpp"
#include "platform/spi/spi_hal_1.hpp"
#include "platform/spi/spi_hal_3.hpp"
#include "platform/spi/spi_master_1.hpp"
#include "platform/spi/spi_master_3.hpp"
#include "platform/timer/advanced_base.hpp"
#include "platform/timer/basic_base.hpp"
#include "platform/timer/general_purpose_base.hpp"
#include "platform/timer/timer_2.hpp"
#include "platform/uart/uart_1.hpp"
#include "platform/uart/uart_2.hpp"
#include "platform/uart/uart_base.hpp"
#include "platform/uart/uart_baudrate.hpp"
#include "platform/uart/uart_hal_1.hpp"
#include "platform/uart/uart_hal_2.hpp"
#endif // MODM_PLATFORM_HPP