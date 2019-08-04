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

#ifndef MODM_STM32_GPIO_PIN_B3_HPP
#define MODM_STM32_GPIO_PIN_B3_HPP

#include "../device.hpp"
#include "base.hpp"
#include "set.hpp"

namespace modm::platform
{

/// @cond
class GpioB3;
using GpioOutputB3 = GpioB3;
using GpioInputB3  = GpioB3;
/// @endcond

/// IO class for Pin B3
/// @ingroup	modm_platform_gpio
class GpioB3 : public Gpio, public ::modm::GpioIO
{
	template<class... Gpios>
	friend class GpioSet;
	using PinSet = GpioSet<GpioB3>;
	friend class Adc;
	friend class Adc1; friend class Adc2;
	friend class Adc3; friend class Adc4;
public:
	using Output = GpioB3;
	using Input = GpioB3;
	using IO = GpioB3;
	using Type = GpioB3;
	static constexpr bool isInverted = false;
	static constexpr Port port = Port::B; ///< Port name
	static constexpr uint8_t pin = 3; ///< Pin number

protected:
	/// Bitmask for registers that contain a 1bit value for every pin.
	static constexpr uint16_t mask  = 0x1 << pin;
	/// Bitmask for registers that contain a 2bit value for every pin.
	static constexpr uint32_t mask2 = 0x3 << (pin * 2);
	/// Port Number.
	static constexpr uint8_t port_nr = uint8_t(port);
	/// Alternate Function register id. 0 for pin 0-7. 1 for pin 8-15.
	static constexpr uint8_t af_id  = pin / 8;
	/// Alternate Function offset.
	static constexpr uint8_t af_offset = (pin * 4) % 32;
	/// Alternate Function register mask.
	static constexpr uint32_t af_mask  = 0xf << af_offset;
	/// ExternalInterruptIRQ
	static constexpr IRQn_Type ExternalInterruptIRQ = EXTI3_IRQn;

public:
	/// @cond
	inline static void setAlternateFunction(uint8_t af) {
		GPIOB->AFR[af_id] = (GPIOB->AFR[af_id] & ~af_mask) | ((af & 0xf) << af_offset);
		GPIOB->MODER = (GPIOB->MODER & ~mask2) | (i(Mode::AlternateFunction) << (pin * 2));
	}

	/// Enable Analog Mode which is needed to use this pin as an ADC input.
	inline static void setAnalogInput() { PinSet::setAnalogInput(); }
	/// @endcond

public:
	// GpioOutput
	// start documentation inherited
	inline static void setOutput() { PinSet::setOutput(); }
	inline static void setOutput(bool status) { PinSet::setOutput(status); }
	inline static void set() { PinSet::set(); }
	inline static void set(bool status) { PinSet::set(status); }
	inline static void reset() { PinSet::reset(); }
	inline static void toggle() {
		if (isSet()) { reset(); }
		else         { set();   }
	}
	inline static bool isSet() { return (GPIOB->ODR & mask); }
	// stop documentation inherited
	inline static void configure(OutputType type, OutputSpeed speed = OutputSpeed::MHz50) { PinSet::configure(type, speed); }
	inline static void setOutput(OutputType type, OutputSpeed speed = OutputSpeed::MHz50) { PinSet::setOutput(type, speed); }
	// GpioInput
	// start documentation inherited
	inline static void setInput() { PinSet::setInput(); }
	inline static bool read() { return (GPIOB->IDR & mask); }
	// end documentation inherited
	inline static void configure(InputType type) { PinSet::configure(type); }
	inline static void setInput(InputType type) { PinSet::setInput(type); }
	// External Interrupts
	// Warning: This will disable any previously enabled interrupt which is
	// routed to the same interupt line, e.g. PA3 will disable PB3.
	// This is a hardware limitation by the STM32 EXTI.
	inline static void enableExternalInterrupt()
	{
		// PA[x], x =  0 ..  3 maps to EXTICR[0]
		// PA[x], x =  4 ..  7 maps to EXTICR[1]
		// PA[x], x =  8 .. 11 maps to EXTICR[2]
		// PA[x], x = 12 .. 15 maps to EXTICR[3]
		// => bit3 and bit2 (mask 0x0c) specify the register
		// => bit1 and bit0 (mask 0x03) specify the bit position
		constexpr uint8_t index   = (pin & 0b1100) >> 2;
		constexpr uint8_t bit_pos = (pin & 0b0011) << 2;
		constexpr uint16_t syscfg_mask = (0b1111) << bit_pos;
		constexpr uint16_t syscfg_value = (port_nr & (0b1111)) << bit_pos;
		SYSCFG->EXTICR[index] = (SYSCFG->EXTICR[index] & ~syscfg_mask) | syscfg_value;
		EXTI->IMR1 |= mask;
	}
	inline static void disableExternalInterrupt() { EXTI->IMR1 &= ~mask; }
	inline static void enableExternalInterruptVector(const uint32_t priority)
	{
		NVIC_SetPriority(ExternalInterruptIRQ, priority);
		NVIC_EnableIRQ(ExternalInterruptIRQ);
	}
	inline static void disableExternalInterruptVector() { NVIC_DisableIRQ(ExternalInterruptIRQ); }
	inline static void setInputTrigger(const InputTrigger trigger)
	{
		switch (trigger)
		{
		case InputTrigger::RisingEdge:
			EXTI->RTSR1 |=  mask;
			EXTI->FTSR1 &= ~mask;
			break;
		case InputTrigger::FallingEdge:
			EXTI->RTSR1 &= ~mask;
			EXTI->FTSR1 |=  mask;
			break;
		case InputTrigger::BothEdges:
			EXTI->RTSR1 |=  mask;
			EXTI->FTSR1 |=  mask;
			break;
		}
	}
	inline static bool getExternalInterruptFlag() { return (EXTI->PR1 & mask); }
	inline static void acknowledgeExternalInterruptFlag() { EXTI->PR1 = mask; }
	// GpioIO
	// start documentation inherited
	inline static Direction getDirection() {
		uint32_t mode = (GPIOB->MODER & mask2);
		if (mode == (i(Mode::Input) << pin * 2)) {
			return Direction::In;
		}
		if (mode == (i(Mode::Output) << pin * 2)) {
			return Direction::Out;
		}
		return Direction::Special;
	}
	// end documentation inherited
	inline static void lock() { PinSet::lock(); }
	inline static void disconnect() {
		setInput(InputType::Floating);
		GPIOB->AFR[af_id] &= ~af_mask;
	}

public:
#ifdef __DOXYGEN__
	/// @{
	/// Connect to any software peripheral
	using BitBang = GpioSignal;
	/// Connect to Tim2
	using Ch2 = GpioSignal;
	/// Connect to Usart1
	using De = GpioSignal;
	/// Connect to Comp2
	using Inm = GpioSignal;
	/// Connect to Sys
	using Jtdo = GpioSignal;
	/// Connect to Usart1
	using Rts = GpioSignal;
	/// Connect to Spi1 or Spi3
	using Sck = GpioSignal;
	/// Connect to Sai1
	using SckB = GpioSignal;
	/// Connect to Sys
	using Swo = GpioSignal;
	/// @}
#endif
	/// @cond
	template< Peripheral peripheral >
	struct BitBang { static void connect();
		static_assert(
			(peripheral == Peripheral::BitBang),
			"GpioB3::BitBang only connects to software drivers!");
	};
	template< Peripheral peripheral >
	struct Ch2 { static void connect();
		static_assert(
			(peripheral == Peripheral::Tim2),
			"GpioB3::Ch2 only connects to Tim2!");
	};
	template< Peripheral peripheral >
	struct De { static void connect();
		static_assert(
			(peripheral == Peripheral::Usart1),
			"GpioB3::De only connects to Usart1!");
	};
	template< Peripheral peripheral >
	struct Inm { static void connect();
		static_assert(
			(peripheral == Peripheral::Comp2),
			"GpioB3::Inm only connects to Comp2!");
	};
	template< Peripheral peripheral >
	struct Jtdo { static void connect();
		static_assert(
			(peripheral == Peripheral::Sys),
			"GpioB3::Jtdo only connects to Sys!");
	};
	template< Peripheral peripheral >
	struct Rts { static void connect();
		static_assert(
			(peripheral == Peripheral::Usart1),
			"GpioB3::Rts only connects to Usart1!");
	};
	template< Peripheral peripheral >
	struct Sck { static void connect();
		static_assert(
			(peripheral == Peripheral::Spi1) ||
			(peripheral == Peripheral::Spi3),
			"GpioB3::Sck only connects to Spi1 or Spi3!");
	};
	template< Peripheral peripheral >
	struct SckB { static void connect();
		static_assert(
			(peripheral == Peripheral::Sai1),
			"GpioB3::SckB only connects to Sai1!");
	};
	template< Peripheral peripheral >
	struct Swo { static void connect();
		static_assert(
			(peripheral == Peripheral::Sys),
			"GpioB3::Swo only connects to Sys!");
	};
	/// @endcond
private:
	template< Peripheral peripheral >
	static constexpr int8_t AdcChannel = -1;
};

/// @cond
template<>
struct GpioB3::BitBang<Peripheral::BitBang>
{
	using Gpio = GpioB3;
	static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang;
	static constexpr int af = -1;
	inline static void connect() {}
};
template<>
struct GpioB3::Ch2<Peripheral::Tim2>
{
	using Gpio = GpioB3;
	static constexpr Gpio::Signal Signal = Gpio::Signal::Ch2;
	static constexpr int af = 1;
	inline static void
	connect()
	{
		setAlternateFunction(1);
	}
};
template<>
struct GpioB3::De<Peripheral::Usart1>
{
	using Gpio = GpioB3;
	static constexpr Gpio::Signal Signal = Gpio::Signal::De;
	static constexpr int af = 7;
	inline static void
	connect()
	{
		setAlternateFunction(7);
	}
};
template<>
struct GpioB3::Inm<Peripheral::Comp2>
{
	using Gpio = GpioB3;
	static constexpr Gpio::Signal Signal = Gpio::Signal::Inm;
	static constexpr int af = -1;
	inline static void
	connect()
	{
		disconnect();
		setAnalogInput();
	}
};
template<>
struct GpioB3::Jtdo<Peripheral::Sys>
{
	using Gpio = GpioB3;
	static constexpr Gpio::Signal Signal = Gpio::Signal::Jtdo;
	static constexpr int af = 0;
	inline static void
	connect()
	{
		setAlternateFunction(0);
	}
};
template<>
struct GpioB3::Rts<Peripheral::Usart1>
{
	using Gpio = GpioB3;
	static constexpr Gpio::Signal Signal = Gpio::Signal::Rts;
	static constexpr int af = 7;
	inline static void
	connect()
	{
		setAlternateFunction(7);
	}
};
template<>
struct GpioB3::Sck<Peripheral::Spi1>
{
	using Gpio = GpioB3;
	static constexpr Gpio::Signal Signal = Gpio::Signal::Sck;
	static constexpr int af = 5;
	inline static void
	connect()
	{
		setAlternateFunction(5);
	}
};
template<>
struct GpioB3::Sck<Peripheral::Spi3>
{
	using Gpio = GpioB3;
	static constexpr Gpio::Signal Signal = Gpio::Signal::Sck;
	static constexpr int af = 6;
	inline static void
	connect()
	{
		setAlternateFunction(6);
	}
};
template<>
struct GpioB3::SckB<Peripheral::Sai1>
{
	using Gpio = GpioB3;
	static constexpr Gpio::Signal Signal = Gpio::Signal::SckB;
	static constexpr int af = 13;
	inline static void
	connect()
	{
		setAlternateFunction(13);
	}
};
template<>
struct GpioB3::Swo<Peripheral::Sys>
{
	using Gpio = GpioB3;
	static constexpr Gpio::Signal Signal = Gpio::Signal::Swo;
	static constexpr int af = 0;
	inline static void
	connect()
	{
		setAlternateFunction(0);
	}
};
/// @endcond

} // namespace modm::platform

#endif // MODM_STM32_GPIO_PIN_B3_HPP