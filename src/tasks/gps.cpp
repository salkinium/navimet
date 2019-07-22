/*
 * Copyright (c) 2018, Niklas Hauser
 *
 * This file is part of the Navimet project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "gps.hpp"
#include "../board.hpp"
#include <modm/processing/timer.hpp>
#include <modm/architecture/interface/assert.hpp>
#include <modm/math/geometry/angle.hpp>
using namespace modm::literals;

#include <ublox/message/CfgPrtUart.h>
#include <ublox/message/CfgRate.h>
#include <ublox/message/NavPosllhPoll.h>
#include <comms/units.h>


#undef	MODM_LOG_LEVEL
#define	MODM_LOG_LEVEL modm::log::DEBUG

namespace
{
using GpsUart = modm::platform::Usart1;
constexpr uint16_t update_period{100}; // ms period, max 10 Hz
modm::ShortPeriodicTimer tmr{update_period};

std::vector<std::uint8_t> in_data;
using AllInMessages = std::tuple<navimet::GpsTask::InNavPosllh>;
using Frame = ublox::frame::UbloxFrame<navimet::GpsTask::InMessage, AllInMessages>;
Frame frame;

float m_latitude{0};
float m_longitude{0};
float m_haccuracy{1e9};
float m_vaccuracy{1e9};

}

namespace navimet
{

void
GpsTask::initialize()
{
	GpsUart::connect<Board::D0::Rx, Board::D1::Tx>(Gpio::InputType::PullUp);

	GpsUart::initialize<Board::SystemClock, 9.6_kBd>();
	GpsUart::discardReceiveBuffer();
	configurePort();
	// Some NEO-6M chips don't support changing their baudrate ;_(
	// GpsUart::initialize<Board::systemClock, 115.2_kBd>();
	// configurePort();

	configureUpdateRate();

	MODM_LOG_INFO << "GPS task initialized." << modm::endl;
}

void
GpsTask::configurePort()
{
	using OutCfgPrtUart = ublox::message::CfgPrtUart<OutMessage>;
	OutCfgPrtUart msg;

	auto& outProtoMaskField = msg.field_outProtoMask();
	outProtoMaskField.setBitValue(outProtoMaskField.BitIdx_outUbx, true);
	outProtoMaskField.setBitValue(outProtoMaskField.BitIdx_outNmea, false);

	auto& inProtoMaskField = msg.field_inProtoMask();
	inProtoMaskField.setBitValue(inProtoMaskField.BitIdx_inUbx, true);
	inProtoMaskField.setBitValue(inProtoMaskField.BitIdx_inNmea, false);

	// This may not work
	msg.field_baudRate().setScaled(9.6_kBd);
	MODM_LOG_INFO << "GPS baudrate set to " << msg.field_baudRate().value() << modm::endl;

	sendMessage(msg);
}

void
GpsTask::configureUpdateRate()
{
	using OutCfgRate = ublox::message::CfgRate<OutMessage>;
	OutCfgRate msg;

	msg.field_measRate().setScaled(update_period);
	MODM_LOG_INFO.printf("GPS measurement rate set to %ums\n", msg.field_measRate().value());

	sendMessage(msg);
}

float
GpsTask::latitude() const
{
	return m_latitude;
}

float
GpsTask::longitude() const
{
	return m_longitude;
}

float
GpsTask::accuracy() const
{
	return m_vaccuracy;
}

float
GpsTask::distance_to(const float latitude, const float longitude) const
{
	static constexpr float R = 6371e3;
	const float dlat = (m_latitude - latitude);
	const float dlon = (m_longitude - longitude);

	const float a = std::sin(dlat/2) * std::sin(dlat/2) +
					std::cos(latitude) * std::cos(m_latitude) *
					std::sin(dlon/2) * std::sin(dlon/2);
	const float c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
	return R * c;
}

float
GpsTask::heading_to(float latitude, float longitude) const
{
	const float dlon = (m_longitude - longitude);
	const float y = std::sin(dlon) * std::cos(m_latitude);
	const float x = std::cos(latitude) * std::sin(m_latitude) -
					std::sin(latitude) * std::cos(m_latitude) * std::cos(dlon);
	const float brng = std::atan2(y, x);
	return std::fmod(brng + M_PI, M_PI);
}

void
GpsTask::handle(InNavPosllh& msg)
{
	const float lat = comms::units::getDegrees<float>(msg.field_lat());
	const float lon = comms::units::getDegrees<float>(msg.field_lon());
	const uint16_t alt = comms::units::getMeters<uint16_t>(msg.field_height());
	const uint16_t hacc = comms::units::getMeters<float>(msg.field_hAcc());
	const uint16_t vacc = comms::units::getMeters<float>(msg.field_vAcc());
	MODM_LOG_INFO.printf("POS: %3.9f %3.9f %um (~%um ~%um)\n", lat, lon, alt, hacc, vacc);

	m_latitude = comms::units::getRadians<float>(msg.field_lat());
	m_longitude = comms::units::getRadians<float>(msg.field_lon());

	m_haccuracy = comms::units::getMeters<float>(msg.field_hAcc());
	m_vaccuracy = comms::units::getMeters<float>(msg.field_vAcc());
}

void
GpsTask::handle(InMessage&)
{}

void
GpsTask::performRead()
{
	if (uint8_t data; GpsUart::read(data))
	{
		in_data.push_back(data);
		processInputData();
	}
}

void
GpsTask::processInputData()
{
	std::size_t consumed = 0U;
	while (consumed < in_data.size()) {
		// Smart pointer to the message object.
		Frame::MsgPtr msgPtr;

		// Type of the message interface class
		using MsgType = Frame::MsgPtr::element_type;

		// Get the iterator for reading
		auto begIter = comms::readIteratorFor<MsgType>(&in_data[0] + consumed);
		auto iter = begIter;

		// Do the read
		auto es = frame.read(msgPtr, iter, in_data.size() - consumed);
		if (es == comms::ErrorStatus::NotEnoughData) {
			break; // Not enough data in the buffer, stop processing
		}

		if (es == comms::ErrorStatus::ProtocolError) {
			// Something is not right with the data, remove one character and try again
			++consumed;
			continue;
		}
		if (es == comms::ErrorStatus::Success) {
			modm_assert(msgPtr, "gps", "ubx", "read"); // If read is successful, msgPtr is expected to hold a valid pointer
			msgPtr->dispatch(*this); // Dispatch message for handling
		}

		// The iterator for reading has been advanced, update the difference
		consumed += std::distance(begIter, iter);
	}

	in_data.erase(in_data.begin(), in_data.begin() + consumed);
}

void
GpsTask::sendMessage(const OutMessage& msg)
{
	OutBuffer buf;
	buf.reserve(frame.length(msg)); // Reserve enough space
	auto iter = std::back_inserter(buf);
	auto es = frame.write(msg, iter, buf.max_size());
	if (es == comms::ErrorStatus::UpdateRequired) {
		auto* updateIter = &buf[0];
		es = frame.update(updateIter, buf.size());
	}
	static_cast<void>(es);
	modm_assert_debug(es == comms::ErrorStatus::Success, "gps", "ubx", "send", es); // do not expect any error

	auto count = GpsUart::write(buf.data(), buf.size());
	// {
	// 	int i{-6};
	// 	for (auto const &data : buf) {
	// 		MODM_LOG_INFO.printf("%2d: %02x\n", i++, data);
	// 	}
	// }

	buf.erase(buf.begin(), buf.begin() + count);
}


bool
GpsTask::update()
{
	PT_BEGIN();

	while (true)
	{
		performRead();

		if (tmr.execute())
		{
			using OutNavPosllhPoll = ublox::message::NavPosllhPoll<OutMessage>;
			sendMessage(OutNavPosllhPoll());
		}

		PT_YIELD();
	}

	PT_END();
}

}
