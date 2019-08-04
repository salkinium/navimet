/*
 * Copyright (c) 2009, Martin Rosekeit
 * Copyright (c) 2009-2012, Fabian Greif
 * Copyright (c) 2011, Georgi Grinshpun
 * Copyright (c) 2012, 2016, Sascha Schade
 * Copyright (c) 2012, 2014-2019, Niklas Hauser
 * Copyright (c) 2013-2014, Kevin Läufer
 * Copyright (c) 2018, Christopher Durand
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "rcc.hpp"
#include "common.hpp"

namespace modm::clock
{
uint32_t modm_fastdata fcpu(16'000'000);
uint32_t modm_fastdata fcpu_kHz(16'000);
uint16_t modm_fastdata fcpu_MHz(16);
uint16_t modm_fastdata ns_per_loop(187);
}

// ----------------------------------------------------------------------------
bool
modm::platform::Rcc::enableInternalClock(uint32_t waitCycles)
{
	bool retval;
	RCC->CR |= RCC_CR_HSION;
	while (not (retval = (RCC->CR & RCC_CR_HSIRDY)) and --waitCycles)
		;
	return retval;
}

bool
modm::platform::Rcc::enableMultiSpeedInternalClock(MsiFrequency msi_frequency, uint32_t waitCycles)
{
	bool retval;
	RCC->CR = (RCC->CR & ~RCC_CR_MSIRANGE) | static_cast<uint32_t>(msi_frequency) | RCC_CR_MSIRGSEL | RCC_CR_MSION;
	while (not (retval = (RCC->CR & RCC_CR_MSIRDY)) and --waitCycles)
		;
	return retval;
}
bool
modm::platform::Rcc::enableExternalClock(uint32_t waitCycles)
{
	bool retval;
	RCC->CR |= RCC_CR_HSEBYP | RCC_CR_HSEON;
	while (not (retval = (RCC->CR & RCC_CR_HSERDY)) and --waitCycles)
		;
	return retval;
}

bool
modm::platform::Rcc::enableExternalCrystal(uint32_t waitCycles)
{
	bool retval;
	RCC->CR = (RCC->CR & ~RCC_CR_HSEBYP) | RCC_CR_HSEON;
	while (not (retval = (RCC->CR & RCC_CR_HSERDY)) and --waitCycles)
		;
	return retval;
}

bool
modm::platform::Rcc::enableLowSpeedInternalClock(uint32_t waitCycles)
{
	bool retval;
	RCC->CSR |= RCC_CSR_LSION;
	while (not (retval = (RCC->CSR & RCC_CSR_LSIRDY)) and --waitCycles)
		;
	return retval;
}

bool
modm::platform::Rcc::enableLowSpeedExternalClock(uint32_t waitCycles)
{
	bool retval;
	RCC->BDCR |= RCC_BDCR_LSEBYP | RCC_BDCR_LSEON;
	while (not (retval = (RCC->BDCR & RCC_BDCR_LSERDY)) and --waitCycles)
		;
	return retval;
}

bool
modm::platform::Rcc::enableLowSpeedExternalCrystal(uint32_t waitCycles)
{
	bool retval;
	RCC->BDCR = (RCC->BDCR & ~RCC_BDCR_LSEBYP) | RCC_BDCR_LSEON;
	while (not (retval = (RCC->BDCR & RCC_BDCR_LSERDY)) and --waitCycles)
		;
	return retval;
}

// ----------------------------------------------------------------------------
bool
modm::platform::Rcc::enablePll(PllSource source,
	uint8_t pllM, uint16_t pllN, uint8_t pllR, uint32_t waitCycles)
{
	// Read reserved values and clear all other values
	uint32_t tmp = RCC->PLLCFGR & ~(
			RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN |
			// RCC_PLLCFGR_PLLPEN | RCC_PLLCFGR_PLLP |
			// RCC_PLLCFGR_PLLQEN | RCC_PLLCFGR_PLLQ |
			RCC_PLLCFGR_PLLREN | RCC_PLLCFGR_PLLR);

	// PLLSRC source for pll
	tmp |= static_cast<uint32_t>(source);

	// PLLM factor is user defined VCO input frequency must be configured between 4MHz and 16Mhz
	tmp |= (uint32_t(pllM - 1) << RCC_PLLCFGR_PLLM_Pos) & RCC_PLLCFGR_PLLM;

	// PLLN factor is user defined: between 64 and 344 MHz
	tmp |= (uint32_t(pllN) << RCC_PLLCFGR_PLLN_Pos) & RCC_PLLCFGR_PLLN;

	// PLLR divider for CPU frequency
	tmp |= ((uint32_t(pllR / 2) - 1) << RCC_PLLCFGR_PLLR_Pos) & RCC_PLLCFGR_PLLR;
	// PLLQ (21) divider for USB frequency; (00: PLLQ = 2, 01: PLLQ = 4, etc.)
	// tmp |= (((uint32_t) (pllQ / 2) - 1) << RCC_PLLCFGR_PLLQ_Pos) & RCC_PLLCFGR_PLLQ;
	// enable pll USB clock output
	// tmp |= RCC_PLLCFGR_PLLQEN;
	// enable pll CPU clock output
	tmp |= RCC_PLLCFGR_PLLREN;

	RCC->PLLCFGR = tmp;

	// enable pll
	RCC->CR |= RCC_CR_PLLON;

	while (not (tmp = (RCC->CR & RCC_CR_PLLRDY)) and --waitCycles)
		;

	return tmp;
}
// ----------------------------------------------------------------------------
bool
modm::platform::Rcc::enableSystemClock(SystemClockSource src, uint32_t waitCycles)
{
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | uint32_t(src);

	// Wait till the main PLL is used as system clock source
	src = SystemClockSource(uint32_t(src) << 2);
	while ((RCC->CFGR & RCC_CFGR_SWS) != uint32_t(src))
	{
		if (not --waitCycles)
			return false;
	}

	return true;
}