/*
 * Copyright (c) 2014-2015, 2018, Daniel Krebs
 * Copyright (c) 2015, Sascha Schade
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_NRF24_CONFIG_HPP
#define MODM_NRF24_CONFIG_HPP

#include <stdint.h>
#include <modm/debug/logger.hpp>

#include "nrf24_phy.hpp"
#include "nrf24_definitions.hpp"

namespace modm
{

/// @ingroup modm_driver_nrf24
struct Nrf24ConfigParameters
{
	enum class
	Mode
	{
		Rx,
		Tx,
		Invalid
	};

	enum class
	Speed : uint32_t
	{
		kBps250 = 250000,
		MBps1 =	1000000,
		MBps2 =	2000000
	};

	enum class
	Crc
	{
		NoCrc = 0,
		Crc1Byte = 1,
		Crc2Byte = 2
	};

	enum class
	AddressWidth : uint8_t
	{
		/* 2 byte works in hardware but is illegal according to datasheet */
		Byte2 = 0x00,
		Byte3 = 0x01,
		Byte4 = 0x02,
		Byte5 = 0x03
	};

	enum class
	RfPower : uint8_t
	{
		Minus18dBm  = 0x00,
		Minus12dBm  = 0x01,
		Minus6dBm   = 0x02,
		dBm0        = 0x03
	};

	enum class
	AutoRetransmitDelay : uint8_t
	{
		us250   = 0x00,
		us500   = 0x01,
		us750   = 0x02,
		us1000  = 0x03,
		us1250  = 0x04,
		us1500  = 0x05,
		us1750  = 0x06,
		us2000  = 0x07,
		us2250  = 0x08,
		us2500  = 0x09,
		us2750  = 0x0A,
		us3000  = 0x0B,
		us3250  = 0x0C,
		us3500  = 0x0D,
		us3750  = 0x0E,
		us4000  = 0x0F,
	};

	enum class
	AutoRetransmitCount : uint8_t
	{
		Disable         = 0x00,
		Retransmit1     = 0x01,
		Retransmit2     = 0x02,
		Retransmit3     = 0x03,
		Retransmit4     = 0x04,
		Retransmit5     = 0x05,
		Retransmit6     = 0x06,
		Retransmit7     = 0x07,
		Retransmit8     = 0x08,
		Retransmit9     = 0x09,
		Retransmit10    = 0x0A,
		Retransmit11    = 0x0B,
		Retransmit12    = 0x0C,
		Retransmit13    = 0x0D,
		Retransmit14    = 0x0E,
		Retransmit15    = 0x0F
	};
};

/**
 * Configuration interface for nRF24L01+
 *
 * This class allows for configuration of some aspects of the nRF24L01+
 * wireless modules. It doesn't implement every aspect, but hopefully
 * all the often used ones.
 *
 * @ingroup	modm_driver_nrf24
 * @author	Daniel Krebs
 */
template<typename Nrf24Phy>
class Nrf24Config : public Nrf24Register, public Nrf24ConfigParameters
{
public:
	static void
	powerUp()
	{ Nrf24Phy::setBits(NrfRegister::CONFIG, Config::PWR_UP); }

	static void
	powerDown()
	{ Nrf24Phy::clearBits(NrfRegister::CONFIG, Config::PWR_UP); }

	static void
	setChannel(uint8_t channel)
	{ Nrf24Phy::writeRegister(NrfRegister::RF_CH, channel); }

	static void
	setMode(Mode mode);

	static void
	setSpeed(Speed speed);

	static Speed
	getSpeed();

	static void
	setCrc(Crc crc);

	static void
	setAddressWidth(AddressWidth width)
	{ Nrf24Phy::writeRegister(NrfRegister::SETUP_AW, static_cast<uint8_t>(width)); }

	static AddressWidth
	getAddressWidth()
	{ return static_cast<AddressWidth>(
		            Nrf24Register::SetupAw::AW &
		            Nrf24Phy::readRegister(NrfRegister::SETUP_AW)); }

	static void
	setRfPower(RfPower power);

	static void
	setAutoRetransmitDelay(AutoRetransmitDelay delay);

	static void
	setAutoRetransmitCount(AutoRetransmitCount count);

	static void
	enableInterrupt(InterruptFlag irq)
	{ Nrf24Phy::clearBits(NrfRegister::CONFIG, irq); }

	static void
	disableInterrupt(InterruptFlag irq)
	{ Nrf24Phy::setBits(NrfRegister::CONFIG, irq); }

	static void
	enableFeatureNoAck()
	{ Nrf24Phy::setBits(NrfRegister::FEATURE, Feature::EN_DYN_ACK); }

	static void
	disableFeatureNoAck()
	{ Nrf24Phy::clearBits(NrfRegister::FEATURE, Feature::EN_DYN_ACK); }

	/** @brief Enable Rx Pipe and set payload width
	 *
	 *  @param pipe     Pipe Number
	 */
	static void
	enablePipe(Pipe_t pipe, bool enableAutoAck);

	/** @brief Disable Rx Pipe and set payload width
	 *
	 *  @param pipe     Pipe Number
	 */
	static void
	disablePipe(Pipe_t pipe)
	{ Nrf24Phy::clearBits(NrfRegister::EN_RX_ADDR, Flags_t(1 << pipe.value)); }

	/// Return number of pipe that has payload available
	static Pipe_t
	getPayloadPipe();

	static uint8_t
	getRetryCount()
	{ return (Nrf24Phy::readRegister(NrfRegister::OBSERVE_TX) &
		      static_cast<uint8_t>(ObserveTx::ARC_CNT)); }

	static constexpr uint32_t
	toNum(Speed speed)
	{ return static_cast<uint32_t>(speed); }

	static constexpr uint8_t
	toNum(AddressWidth addressWidth)
	{ return (static_cast<uint8_t>(addressWidth) + 2); }

	static constexpr uint8_t
	toNum(Crc crc)
	{ return static_cast<uint8_t>(crc); }

public:
	static inline Mode currentMode{Mode::Invalid};
};

}   // namespace modm

#include "nrf24_config_impl.hpp"

#endif /* MODM_NRF24_CONFIG_HPP */
