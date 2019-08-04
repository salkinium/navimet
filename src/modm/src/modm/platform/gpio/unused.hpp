/*
 * Copyright (c) 2017-2018, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_GPIO_PIN_UNUSED_HPP
#define MODM_STM32_GPIO_PIN_UNUSED_HPP

#include "base.hpp"
#include <modm/architecture/interface/gpio.hpp>

namespace modm
{

namespace platform
{

/**
 * Dummy implementation of an I/O pin.
 *
 * This class can be used when a pin is not required. All functions
 * are dummy functions which do nothing. `read()` will always
 * return `false`.
 *
 * For example when creating a software SPI with the modm::SoftwareSimpleSpi
 * class and the return channel (MISO - Master In Slave Out) is not needed,
 * a good way is to use this class as a parameter when defining the
 * SPI class.
 *
 * Example:
 * @code
 * #include <modm/architecture/platform.hpp>
 *
 * namespace pin
 * {
 *     typedef GpioOutputD7 Clk;
 *     typedef GpioOutputD5 Mosi;
 * }
 *
 * modm::SoftwareSpiMaster< pin::Clk, pin::Mosi, GpioUnused > Spi;
 *
 * ...
 * Spi::write(0xaa);
 * @endcode
 *
 * @author	Fabian Greif
 * @author	Niklas Hauser
 * @ingroup	modm_platform_gpio
 */
class GpioUnused : public Gpio, public ::modm::GpioIO
{
public:
	using Output = GpioUnused;
	using Input = GpioUnused;
	using IO = GpioUnused;
	using Type = GpioUnused;
	static constexpr bool isInverted = false;
	static constexpr Port port = Port(-1);
	static constexpr uint8_t pin = uint8_t(-1);
	static constexpr uint16_t mask = 0;

protected:
	/// @cond
	static void setAlternateFunction(uint8_t) {}
	static void setAnalogInput() {}
	/// @endcond

public:
	// GpioOutput
	// start documentation inherited
	static void setOutput() {}
	static void setOutput(bool) {}
	static void set() {}
	static void set(bool) {}
	static void reset() {}
	static void toggle() {}
	static bool isSet() { return false; }
	// stop documentation inherited
	static void configure(OutputType, OutputSpeed = OutputSpeed::MHz50) {}
	static void setOutput(OutputType, OutputSpeed = OutputSpeed::MHz50) {}

	// GpioInput
	// start documentation inherited
	static void setInput() {}
	static bool read() { return false; }
	// end documentation inherited
	static void configure(InputType) {}
	static void setInput(InputType) {}
	// External Interrupts
	static void enableExternalInterrupt() {}
	static void disableExternalInterrupt() {}
	static void enableExternalInterruptVector(const uint32_t) {}
	static void disableExternalInterruptVector() {}
	static void setInputTrigger(const InputTrigger) {}
	static bool getExternalInterruptFlag() { return false; }
	/// Reset the interrupt flag in the interrupt routine.
	static void acknowledgeExternalInterruptFlag() {}

	// GpioIO
	// start documentation inherited
	static Direction getDirection() { return Direction::Special; }
	// end documentation inherited
	static void lock() {}
	static void disconnect() {}

public:
	/// @cond
	template< Peripheral _ >
	struct Bk1Io0
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Bk1Io0;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Bk1Io1
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Bk1Io1;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Bk1Io2
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Bk1Io2;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Bk1Io3
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Bk1Io3;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Bk1Ncs
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Bk1Ncs;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Bkin
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Bkin;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Bkin2
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Bkin2;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Bkin2Comp1
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Bkin2Comp1;
		static void connect() {}
	};
	template< Peripheral _ >
	struct BkinComp2
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::BkinComp2;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Ch1
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Ch1n
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1n;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Ch2
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Ch2;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Ch2n
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Ch2n;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Ch3
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Ch3;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Ch3n
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Ch3n;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Ch4
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Ch4;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Ck
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Ck;
		static void connect() {}
	};
	template< Peripheral _ >
	struct CkIn
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::CkIn;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Clk
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Clk;
		static void connect() {}
	};
	template< Peripheral _ >
	struct CrsSync
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::CrsSync;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Cts
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Cts;
		static void connect() {}
	};
	template< Peripheral _ >
	struct De
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::De;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Dm
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Dm;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Dp
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Dp;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Etr
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Etr;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Extclk
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Extclk;
		static void connect() {}
	};
	template< Peripheral _ >
	struct FsA
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::FsA;
		static void connect() {}
	};
	template< Peripheral _ >
	struct FsB
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::FsB;
		static void connect() {}
	};
	template< Peripheral _ >
	struct G2Io1
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::G2Io1;
		static void connect() {}
	};
	template< Peripheral _ >
	struct G2Io2
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::G2Io2;
		static void connect() {}
	};
	template< Peripheral _ >
	struct G2Io3
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::G2Io3;
		static void connect() {}
	};
	template< Peripheral _ >
	struct G2Io4
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::G2Io4;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In1
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In1;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In10
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In10;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In11
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In11;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In12
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In12;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In15
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In15;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In16
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In16;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In2
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In2;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In5
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In5;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In6
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In6;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In7
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In7;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In8
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In8;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In9
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In9;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Inm
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Inm;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Inp
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Inp;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Io
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Io;
		static void connect() {}
	};
	template< Peripheral _ >
	struct IrOut
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::IrOut;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Jtck
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Jtck;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Jtdi
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Jtdi;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Jtdo
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Jtdo;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Jtms
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Jtms;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Jtrst
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Jtrst;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Lsco
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Lsco;
		static void connect() {}
	};
	template< Peripheral _ >
	struct MclkA
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::MclkA;
		static void connect() {}
	};
	template< Peripheral _ >
	struct MclkB
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::MclkB;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Mco
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Mco;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Miso
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Miso;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Mosi
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Mosi;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Noe
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Noe;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Nss
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Nss;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Osc32In
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Osc32In;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Osc32Out
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Osc32Out;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Out
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Out;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Out1
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Out1;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Out2
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Out2;
		static void connect() {}
	};
	template< Peripheral _ >
	struct PvdIn
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::PvdIn;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Rts
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Rts;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Rx
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Rx;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Sck
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Sck;
		static void connect() {}
	};
	template< Peripheral _ >
	struct SckA
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::SckA;
		static void connect() {}
	};
	template< Peripheral _ >
	struct SckB
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::SckB;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Scl
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Scl;
		static void connect() {}
	};
	template< Peripheral _ >
	struct SdA
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::SdA;
		static void connect() {}
	};
	template< Peripheral _ >
	struct SdB
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::SdB;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Sda
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Sda;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Smba
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Smba;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Suspend
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Suspend;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Swclk
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Swclk;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Swdio
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Swdio;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Swo
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Swo;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Tamp2
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Tamp2;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Tx
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Tx;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Vinm
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Vinm;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Vinp
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Vinp;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Vout
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Vout;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Wkup1
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Wkup1;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Wkup4
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Wkup4;
		static void connect() {}
	};
	/// @endcond
};

} // namespace platform

} // namespace modm

#endif // MODM_STM32_GPIO_PIN_UNUSED_HPP