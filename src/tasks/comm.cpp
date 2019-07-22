/*
 * Copyright (c) 2019, Niklas Hauser
 *
 * This file is part of the Navimet project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "comm.hpp"
#include "../board.hpp"
#include <modm/processing/timer.hpp>
#include <modm/driver/radio/nrf24.hpp>

using namespace modm::literals;

#undef	MODM_LOG_LEVEL
#define	MODM_LOG_LEVEL modm::log::DEBUG

namespace
{
class ClockUs
{
public:
	typedef uint32_t Type;
	using ClockUsTimer = Timer2;

public:
	template< class SystemClock >
	static void
	enable()
	{
		ClockUsTimer::enable();
		ClockUsTimer::setMode(ClockUsTimer::Mode::UpCounter);
		ClockUsTimer::setPrescaler(SystemClock::Timer2 / 1_MHz);
		ClockUsTimer::setOverflow((Type)-1);
		ClockUsTimer::applyAndReset();

		setTime(0);
		ClockUsTimer::start();
	}

	template< typename TimestampType = modm::Timestamp >
	static TimestampType
	now()
	{ return TimestampType(ClockUsTimer::getValue()); }

	static void
	setTime(const Type time)
	{ ClockUsTimer::setValue(time); }
};

using RadioSpi = SpiMaster1;
using RadioSck = Board::A4;
using RadioMi  = Board::A5;
using RadioMo  = Board::A6;

using RadioCsn = Board::A1;
using RadioCe  = Board::A2;
using RadioIrq = Board::A3;

using NrfPhy    = modm::Nrf24Phy<RadioSpi, RadioCsn, RadioCe>;
using NrfConfig = modm::Nrf24Config<NrfPhy>;
using NrfData   = modm::Nrf24Data<NrfPhy, ClockUs>;

static constexpr uint32_t nrf_magic{0xc0ffeeee};

bool new_position{false};
float plat, plon;

struct nrf_message
{
	nrf_message() = default;
	nrf_message(float lat, float lon): latitude(lat), longitude(lon) {}
	bool isValid() const { return magic == nrf_magic; }
private:
	uint32_t magic{nrf_magic};
public:
	float latitude{0};
	float longitude{0};
} modm_packed;
static_assert(sizeof(nrf_message) <= NrfData::Phy::getMaxPayload(),
              "nrf_message is too large for one NRF24 message!");
}

MODM_ISR(EXTI4) // From Board::A3
{
	RadioIrq::acknowledgeExternalInterruptFlag();
	NrfData::interruptHandler();
}

namespace navimet
{

void
CommTask::initialize()
{
	ClockUs::enable<Board::SystemClock>();
	RadioCsn::setOutput(modm::Gpio::High);
	RadioCe::setOutput(modm::Gpio::Low);

	RadioSpi::connect<RadioMo::Mosi, RadioMi::Miso, RadioSck::Sck>();
	RadioSpi::initialize<Board::SystemClock, 1.6_MHz>();
	MODM_LOG_DEBUG << "COMM SPI initialized." << modm::endl;

	// Initialize radio layers
	NrfData::initialize(0xdeadbeef00, 0x33, 0xFF);

	NrfConfig::setChannel(40);
	NrfConfig::setAutoRetransmitCount(NrfConfig::AutoRetransmitCount::Retransmit3);
	NrfConfig::setAutoRetransmitDelay(NrfConfig::AutoRetransmitDelay::us500);
	NrfConfig::setSpeed(NrfConfig::Speed::MBps1);
	NrfConfig::setCrc(NrfConfig::Crc::Crc2Byte);

	RadioIrq::setInput(RadioIrq::InputType::PullUp);
	RadioIrq::setInputTrigger(RadioIrq::InputTrigger::FallingEdge);
	RadioIrq::enableExternalInterrupt();
	RadioIrq::enableExternalInterruptVector(4);

	MODM_LOG_INFO << "COMM initialized." << modm::endl;
}

void
CommTask::sendPosition(float lat, float lon)
{
	nrf_message data{lat, lon};
	NrfData::Packet packet;
	packet.setDestination(0xff);
	memcpy(packet.payload, &data, sizeof(nrf_message));
	NrfData::sendPacket(packet);
	MODM_LOG_DEBUG.printf("COMM: Broadcasting position: %3.4f %3.4f\n",
	                      (double)lat, (double)lon);
}

bool
CommTask::receivePosition(float *lat, float *lon)
{
	if (new_position) {
		*lat = plat;
		*lon = plon;
		new_position = false;
		return true;
	}
	return false;
}

bool
CommTask::update()
{
	PT_BEGIN();

	while (true)
	{
		NrfData::update();

		if (NrfData::Packet packet; NrfData::getPacket(packet))
		{
			nrf_message data;
			memcpy(&data, packet.payload, sizeof(nrf_message));
			if (data.isValid())
			{
				plat = data.latitude;
				plon = data.longitude;
				new_position = true;
				MODM_LOG_DEBUG.printf("COMM: Received from 0x%02x: %3.4f %3.4f\n",
				                      packet.getSource(), (double)plat, (double)plon);
			}
		}

		PT_YIELD();
	}

	PT_END();
}

}
