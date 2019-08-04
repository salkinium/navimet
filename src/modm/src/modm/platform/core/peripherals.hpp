/*
 * Copyright (c) 2019, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_PERIPHERALS_HPP
#define MODM_STM32_PERIPHERALS_HPP

namespace modm::platform
{

enum class
Peripheral
{
	BitBang,
	Adc1,
	Can1,
	Comp1,
	Comp2,
	Crc,
	Dac1,
	Dma1,
	Dma2,
	Flash,
	I2c1,
	I2c3,
	Irtim,
	Iwdg,
	Lptim1,
	Lptim2,
	Lpuart1,
	None,
	Opamp1,
	Quadspi,
	Rcc,
	Rng,
	Rtc,
	Sai1,
	Spi1,
	Spi3,
	Swpmi1,
	Sys,
	Tim1,
	Tim15,
	Tim16,
	Tim2,
	Tim6,
	Tim7,
	Tsc,
	Usart1,
	Usart2,
	Usb,
	Wwdg,
	Syscfg = Sys,
};

}

#endif // MODM_STM32_PERIPHERALS_HPP