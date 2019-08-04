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

#include "common.hpp"

namespace modm::platform
{

constexpr Rcc::flash_latency
Rcc::computeFlashLatency(uint32_t Core_Hz, uint16_t Core_mV)
{
	constexpr uint32_t flash_latency_1000[] =
	{
		6000000,
		12000000,
		18000000,
		26000000,
	};
	constexpr uint32_t flash_latency_1200[] =
	{
		16000000,
		32000000,
		48000000,
		64000000,
		80000000,
	};
	const uint32_t *lut(flash_latency_1000);
	uint8_t lut_size(sizeof(flash_latency_1000) / sizeof(uint32_t));
	// find the right table for the voltage
	if (1200 <= Core_mV) {
		lut = flash_latency_1200;
		lut_size = sizeof(flash_latency_1200) / sizeof(uint32_t);
	}
	// find the next highest frequency in the table
	uint8_t latency(0);
	uint32_t max_freq(0);
	while (latency < lut_size)
	{
		if (Core_Hz <= (max_freq = lut[latency]))
			break;
		latency++;
	}
	return {latency, max_freq};
}

template< uint32_t Core_Hz, uint16_t Core_mV = 3300 >
uint32_t
Rcc::setFlashLatency()
{
	constexpr flash_latency fl = computeFlashLatency(Core_Hz, Core_mV);
	static_assert(Core_Hz <= fl.max_frequency, "CPU Frequency is too high for this core voltage!");

	uint32_t acr = FLASH->ACR & ~FLASH_ACR_LATENCY;
	// set flash latency
	acr |= fl.latency;
	// enable flash prefetch and data and instruction cache
	acr |= FLASH_ACR_PRFTEN | FLASH_ACR_DCEN | FLASH_ACR_ICEN;
	FLASH->ACR = acr;
	return fl.max_frequency;
}

template< uint32_t Core_Hz >
void
Rcc::updateCoreFrequency()
{
	modm::clock::fcpu     = Core_Hz;
	modm::clock::fcpu_kHz = Core_Hz / 1'000;
	modm::clock::fcpu_MHz = Core_Hz / 1'000'000;
	modm::clock::ns_per_loop = ::round(3000.f / (Core_Hz / 1'000'000));
}

constexpr bool
rcc_check_enable(Peripheral peripheral)
{
	switch(peripheral) {
		case Peripheral::Adc1:
		case Peripheral::Can1:
		case Peripheral::Crc:
		case Peripheral::Dac1:
		case Peripheral::Dma1:
		case Peripheral::Dma2:
		case Peripheral::Flash:
		case Peripheral::I2c1:
		case Peripheral::I2c3:
		case Peripheral::Lptim1:
		case Peripheral::Lptim2:
		case Peripheral::Lpuart1:
		case Peripheral::Rng:
		case Peripheral::Rtc:
		case Peripheral::Sai1:
		case Peripheral::Spi1:
		case Peripheral::Spi3:
		case Peripheral::Swpmi1:
		case Peripheral::Tim1:
		case Peripheral::Tim15:
		case Peripheral::Tim16:
		case Peripheral::Tim2:
		case Peripheral::Tim6:
		case Peripheral::Tim7:
		case Peripheral::Tsc:
		case Peripheral::Usart1:
		case Peripheral::Usart2:
		case Peripheral::Wwdg:
			return true;
		default:
			return false;
	}
}

template< Peripheral peripheral >
void
Rcc::enable()
{
	static_assert(rcc_check_enable(peripheral),
		"Rcc::enable() doesn't know this peripheral!");

	__DSB();
	if constexpr (peripheral == Peripheral::Adc1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN; __DSB();
			RCC->AHB2RSTR |= RCC_AHB2RSTR_ADCRST; __DSB();
			RCC->AHB2RSTR &= ~RCC_AHB2RSTR_ADCRST;
		}
	if constexpr (peripheral == Peripheral::Can1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_CAN1EN; __DSB();
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_CAN1RST; __DSB();
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_CAN1RST;
		}
	if constexpr (peripheral == Peripheral::Crc)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN; __DSB();
			RCC->AHB1RSTR |= RCC_AHB1RSTR_CRCRST; __DSB();
			RCC->AHB1RSTR &= ~RCC_AHB1RSTR_CRCRST;
		}
	if constexpr (peripheral == Peripheral::Dac1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_DAC1EN; __DSB();
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_DAC1RST; __DSB();
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_DAC1RST;
		}
	if constexpr (peripheral == Peripheral::Dma1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN; __DSB();
			RCC->AHB1RSTR |= RCC_AHB1RSTR_DMA1RST; __DSB();
			RCC->AHB1RSTR &= ~RCC_AHB1RSTR_DMA1RST;
		}
	if constexpr (peripheral == Peripheral::Dma2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; __DSB();
			RCC->AHB1RSTR |= RCC_AHB1RSTR_DMA2RST; __DSB();
			RCC->AHB1RSTR &= ~RCC_AHB1RSTR_DMA2RST;
		}
	if constexpr (peripheral == Peripheral::Flash)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB1ENR |= RCC_AHB1ENR_FLASHEN; __DSB();
			RCC->AHB1RSTR |= RCC_AHB1RSTR_FLASHRST; __DSB();
			RCC->AHB1RSTR &= ~RCC_AHB1RSTR_FLASHRST;
		}
	if constexpr (peripheral == Peripheral::I2c1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN; __DSB();
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C1RST; __DSB();
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C1RST;
		}
	if constexpr (peripheral == Peripheral::I2c3)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_I2C3EN; __DSB();
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C3RST; __DSB();
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C3RST;
		}
	if constexpr (peripheral == Peripheral::Lptim1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN; __DSB();
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_LPTIM1RST; __DSB();
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_LPTIM1RST;
		}
	if constexpr (peripheral == Peripheral::Lptim2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR2 |= RCC_APB1ENR2_LPTIM2EN; __DSB();
			RCC->APB1RSTR2 |= RCC_APB1RSTR2_LPTIM2RST; __DSB();
			RCC->APB1RSTR2 &= ~RCC_APB1RSTR2_LPTIM2RST;
		}
	if constexpr (peripheral == Peripheral::Lpuart1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN; __DSB();
			RCC->APB1RSTR2 |= RCC_APB1RSTR2_LPUART1RST; __DSB();
			RCC->APB1RSTR2 &= ~RCC_APB1RSTR2_LPUART1RST;
		}
	if constexpr (peripheral == Peripheral::Rng)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN; __DSB();
			RCC->AHB2RSTR |= RCC_AHB2RSTR_RNGRST; __DSB();
			RCC->AHB2RSTR &= ~RCC_AHB2RSTR_RNGRST;
		}
	if constexpr (peripheral == Peripheral::Rtc)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->BDCR |= RCC_BDCR_RTCEN;
		}
	if constexpr (peripheral == Peripheral::Sai1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_SAI1EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_SAI1RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_SAI1RST;
		}
	if constexpr (peripheral == Peripheral::Spi1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
		}
	if constexpr (peripheral == Peripheral::Spi3)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_SPI3EN; __DSB();
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_SPI3RST; __DSB();
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_SPI3RST;
		}
	if constexpr (peripheral == Peripheral::Swpmi1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR2 |= RCC_APB1ENR2_SWPMI1EN; __DSB();
			RCC->APB1RSTR2 |= RCC_APB1RSTR2_SWPMI1RST; __DSB();
			RCC->APB1RSTR2 &= ~RCC_APB1RSTR2_SWPMI1RST;
		}
	if constexpr (peripheral == Peripheral::Tim1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST;
		}
	if constexpr (peripheral == Peripheral::Tim15)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_TIM15EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_TIM15RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM15RST;
		}
	if constexpr (peripheral == Peripheral::Tim16)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_TIM16EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_TIM16RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM16RST;
		}
	if constexpr (peripheral == Peripheral::Tim2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; __DSB();
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_TIM2RST; __DSB();
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM2RST;
		}
	if constexpr (peripheral == Peripheral::Tim6)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN; __DSB();
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_TIM6RST; __DSB();
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM6RST;
		}
	if constexpr (peripheral == Peripheral::Tim7)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN; __DSB();
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_TIM7RST; __DSB();
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM7RST;
		}
	if constexpr (peripheral == Peripheral::Tsc)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB1ENR |= RCC_AHB1ENR_TSCEN; __DSB();
			RCC->AHB1RSTR |= RCC_AHB1RSTR_TSCRST; __DSB();
			RCC->AHB1RSTR &= ~RCC_AHB1RSTR_TSCRST;
		}
	if constexpr (peripheral == Peripheral::Usart1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_USART1EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
		}
	if constexpr (peripheral == Peripheral::Usart2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN; __DSB();
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_USART2RST; __DSB();
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_USART2RST;
		}
	if constexpr (peripheral == Peripheral::Wwdg)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_WWDGEN;
		}
	__DSB();
}

template< Peripheral peripheral >
void
Rcc::disable()
{
	static_assert(rcc_check_enable(peripheral),
		"Rcc::disable() doesn't know this peripheral!");

	__DSB();
	if constexpr (peripheral == Peripheral::Adc1)
		RCC->AHB2ENR &= ~RCC_AHB2ENR_ADCEN;
	if constexpr (peripheral == Peripheral::Can1)
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_CAN1EN;
	if constexpr (peripheral == Peripheral::Crc)
		RCC->AHB1ENR &= ~RCC_AHB1ENR_CRCEN;
	if constexpr (peripheral == Peripheral::Dac1)
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_DAC1EN;
	if constexpr (peripheral == Peripheral::Dma1)
		RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA1EN;
	if constexpr (peripheral == Peripheral::Dma2)
		RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA2EN;
	if constexpr (peripheral == Peripheral::Flash)
		RCC->AHB1ENR &= ~RCC_AHB1ENR_FLASHEN;
	if constexpr (peripheral == Peripheral::I2c1)
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_I2C1EN;
	if constexpr (peripheral == Peripheral::I2c3)
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_I2C3EN;
	if constexpr (peripheral == Peripheral::Lptim1)
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_LPTIM1EN;
	if constexpr (peripheral == Peripheral::Lptim2)
		RCC->APB1ENR2 &= ~RCC_APB1ENR2_LPTIM2EN;
	if constexpr (peripheral == Peripheral::Lpuart1)
		RCC->APB1ENR2 &= ~RCC_APB1ENR2_LPUART1EN;
	if constexpr (peripheral == Peripheral::Rng)
		RCC->AHB2ENR &= ~RCC_AHB2ENR_RNGEN;
	if constexpr (peripheral == Peripheral::Rtc)
		RCC->BDCR &= ~RCC_BDCR_RTCEN;
	if constexpr (peripheral == Peripheral::Sai1)
		RCC->APB2ENR &= ~RCC_APB2ENR_SAI1EN;
	if constexpr (peripheral == Peripheral::Spi1)
		RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
	if constexpr (peripheral == Peripheral::Spi3)
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_SPI3EN;
	if constexpr (peripheral == Peripheral::Swpmi1)
		RCC->APB1ENR2 &= ~RCC_APB1ENR2_SWPMI1EN;
	if constexpr (peripheral == Peripheral::Tim1)
		RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN;
	if constexpr (peripheral == Peripheral::Tim15)
		RCC->APB2ENR &= ~RCC_APB2ENR_TIM15EN;
	if constexpr (peripheral == Peripheral::Tim16)
		RCC->APB2ENR &= ~RCC_APB2ENR_TIM16EN;
	if constexpr (peripheral == Peripheral::Tim2)
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM2EN;
	if constexpr (peripheral == Peripheral::Tim6)
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM6EN;
	if constexpr (peripheral == Peripheral::Tim7)
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM7EN;
	if constexpr (peripheral == Peripheral::Tsc)
		RCC->AHB1ENR &= ~RCC_AHB1ENR_TSCEN;
	if constexpr (peripheral == Peripheral::Usart1)
		RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
	if constexpr (peripheral == Peripheral::Usart2)
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_USART2EN;
	if constexpr (peripheral == Peripheral::Wwdg)
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_WWDGEN;
	__DSB();
}

template< Peripheral peripheral >
bool
Rcc::isEnabled()
{
	static_assert(rcc_check_enable(peripheral),
		"Rcc::isEnabled() doesn't know this peripheral!");

	if constexpr (peripheral == Peripheral::Adc1)
		return RCC->AHB2ENR & RCC_AHB2ENR_ADCEN;
	if constexpr (peripheral == Peripheral::Can1)
		return RCC->APB1ENR1 & RCC_APB1ENR1_CAN1EN;
	if constexpr (peripheral == Peripheral::Crc)
		return RCC->AHB1ENR & RCC_AHB1ENR_CRCEN;
	if constexpr (peripheral == Peripheral::Dac1)
		return RCC->APB1ENR1 & RCC_APB1ENR1_DAC1EN;
	if constexpr (peripheral == Peripheral::Dma1)
		return RCC->AHB1ENR & RCC_AHB1ENR_DMA1EN;
	if constexpr (peripheral == Peripheral::Dma2)
		return RCC->AHB1ENR & RCC_AHB1ENR_DMA2EN;
	if constexpr (peripheral == Peripheral::Flash)
		return RCC->AHB1ENR & RCC_AHB1ENR_FLASHEN;
	if constexpr (peripheral == Peripheral::I2c1)
		return RCC->APB1ENR1 & RCC_APB1ENR1_I2C1EN;
	if constexpr (peripheral == Peripheral::I2c3)
		return RCC->APB1ENR1 & RCC_APB1ENR1_I2C3EN;
	if constexpr (peripheral == Peripheral::Lptim1)
		return RCC->APB1ENR1 & RCC_APB1ENR1_LPTIM1EN;
	if constexpr (peripheral == Peripheral::Lptim2)
		return RCC->APB1ENR2 & RCC_APB1ENR2_LPTIM2EN;
	if constexpr (peripheral == Peripheral::Lpuart1)
		return RCC->APB1ENR2 & RCC_APB1ENR2_LPUART1EN;
	if constexpr (peripheral == Peripheral::Rng)
		return RCC->AHB2ENR & RCC_AHB2ENR_RNGEN;
	if constexpr (peripheral == Peripheral::Rtc)
		return RCC->BDCR & RCC_BDCR_RTCEN;
	if constexpr (peripheral == Peripheral::Sai1)
		return RCC->APB2ENR & RCC_APB2ENR_SAI1EN;
	if constexpr (peripheral == Peripheral::Spi1)
		return RCC->APB2ENR & RCC_APB2ENR_SPI1EN;
	if constexpr (peripheral == Peripheral::Spi3)
		return RCC->APB1ENR1 & RCC_APB1ENR1_SPI3EN;
	if constexpr (peripheral == Peripheral::Swpmi1)
		return RCC->APB1ENR2 & RCC_APB1ENR2_SWPMI1EN;
	if constexpr (peripheral == Peripheral::Tim1)
		return RCC->APB2ENR & RCC_APB2ENR_TIM1EN;
	if constexpr (peripheral == Peripheral::Tim15)
		return RCC->APB2ENR & RCC_APB2ENR_TIM15EN;
	if constexpr (peripheral == Peripheral::Tim16)
		return RCC->APB2ENR & RCC_APB2ENR_TIM16EN;
	if constexpr (peripheral == Peripheral::Tim2)
		return RCC->APB1ENR1 & RCC_APB1ENR1_TIM2EN;
	if constexpr (peripheral == Peripheral::Tim6)
		return RCC->APB1ENR1 & RCC_APB1ENR1_TIM6EN;
	if constexpr (peripheral == Peripheral::Tim7)
		return RCC->APB1ENR1 & RCC_APB1ENR1_TIM7EN;
	if constexpr (peripheral == Peripheral::Tsc)
		return RCC->AHB1ENR & RCC_AHB1ENR_TSCEN;
	if constexpr (peripheral == Peripheral::Usart1)
		return RCC->APB2ENR & RCC_APB2ENR_USART1EN;
	if constexpr (peripheral == Peripheral::Usart2)
		return RCC->APB1ENR1 & RCC_APB1ENR1_USART2EN;
	if constexpr (peripheral == Peripheral::Wwdg)
		return RCC->APB1ENR1 & RCC_APB1ENR1_WWDGEN;
}

}   // namespace modm::platform
