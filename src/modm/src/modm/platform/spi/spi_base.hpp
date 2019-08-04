/*
 * Copyright (c) 2013, Kevin Läufer
 * Copyright (c) 2013-2017, Niklas Hauser
 * Copyright (c) 2014, Daniel Krebs
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_SPI_BASE_HPP
#define MODM_STM32_SPI_BASE_HPP

#include <stdint.h>
#include "../device.hpp"
#include <modm/architecture/interface/register.hpp>

namespace modm
{

namespace platform
{

/**
 * Base class for the SPI classes
 *
 * Provides some common enum that do not depend on the specific SPI.
 *
 * @author Kevin Laeufer
 * @ingroup	modm_platform_spi
 */
class SpiBase
{
public:
	enum class
	Interrupt : uint32_t
	{
		RxBufferNotEmpty	= SPI_CR2_RXNEIE,
		TxBufferEmpty		= SPI_CR2_TXEIE,
		Error				= SPI_CR2_ERRIE,
	};
	MODM_FLAGS32(Interrupt);

	enum class
	InterruptFlag : uint32_t
	{
		TxBufferEmpty		= SPI_SR_TXE,
		RxBufferNotEmpty	= SPI_SR_RXNE,
		CrcError			= SPI_SR_CRCERR,
		ModeFaultError		= SPI_SR_MODF,
		OverrunError		= SPI_SR_OVR,
		Busy				= SPI_SR_BSY,
		FrameFormatError	= SPI_SR_FRE,
		FifoRxLevel			= SPI_SR_FRLVL,
		FifoTxLevel			= SPI_SR_FTLVL,
	};
	MODM_FLAGS32(InterruptFlag);

	enum class
	MasterSelection : uint32_t
	{
		Slave 	= 0,			///< Configure SPI as Slave
		Master 	= SPI_CR1_MSTR,	///< Configure SPI as Master
		All 	= Master,
	};

	enum class
	DataMode : uint32_t
	{
		Mode0 = 0b00,			///< clock normal,   sample on rising  edge
		Mode1 = SPI_CR1_CPHA,	///< clock normal,   sample on falling edge
		Mode2 = SPI_CR1_CPOL,	///< clock inverted, sample on falling  edge
		Mode3 = SPI_CR1_CPOL | SPI_CR1_CPHA,
		///< clock inverted, sample on rising edge
		All = Mode3
	};

	enum class
	DataOrder : uint32_t
	{
		MsbFirst = 0b0,
		LsbFirst = SPI_CR1_LSBFIRST,
		All = LsbFirst,
	};

	enum class
	DataSize : uint32_t
	{
		Bit4  = SPI_CR2_DS_1 | SPI_CR2_DS_0,				///< 0b0011
		Bit5  = SPI_CR2_DS_2,								///< 0b0100
		Bit6  = SPI_CR2_DS_2 | SPI_CR2_DS_0,				///< 0b0101
		Bit7  = SPI_CR2_DS_2 | SPI_CR2_DS_1,				///< 0b0110
		Bit8  = SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0,	///< 0b0111
		Bit9  = SPI_CR2_DS_3,								///< 0b1000
		Bit10 = SPI_CR2_DS_3 | SPI_CR2_DS_0,				///< 0b1001
		Bit11 = SPI_CR2_DS_3 | SPI_CR2_DS_1,				///< 0b1010
		Bit12 = SPI_CR2_DS_3 | SPI_CR2_DS_1 | SPI_CR2_DS_0,	///< 0b1011
		Bit13 = SPI_CR2_DS_3 | SPI_CR2_DS_2,				///< 0b1100
		Bit14 = SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_0,	///< 0b1101
		Bit15 = SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1,	///< 0b1110
		Bit16 = SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0,///< 0b1111
		All   = Bit16,
	};

	enum class
	Prescaler : uint32_t
	{
		Div2 	= 0,
		Div4 	= SPI_CR1_BR_0,
		Div8 	= SPI_CR1_BR_1,
		Div16 	= SPI_CR1_BR_1 | SPI_CR1_BR_0,
		Div32 	= SPI_CR1_BR_2,
		Div64 	= SPI_CR1_BR_2 | SPI_CR1_BR_0,
		Div128 	= SPI_CR1_BR_2 | SPI_CR1_BR_1,
		Div256 	= SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0,
	};

};

} // namespace platform

} // namespace modm

#endif // MODM_STM32_SPI_BASE_HPP