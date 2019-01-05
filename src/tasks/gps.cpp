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
#include <modm/board.hpp>
#include <modm/architecture/interface/assert.hpp>

#include <ublox/message/CfgPrtUart.h>
#include <ublox/message/CfgRate.h>
#include <ublox/message/NavPosllhPoll.h>
#include <comms/units.h>


#undef	MODM_LOG_LEVEL
#define	MODM_LOG_LEVEL modm::log::DEBUG

using GpsUart = modm::platform::Usart1;

namespace navimet
{

void
GpsTask::initialize()
{
	GpsUart::connect<Board::D2::Rx, Board::D8::Tx>(Gpio::InputType::PullUp);

	GpsUart::initialize<Board::systemClock, 9'600>();
	configurePort();
	// Some NEO-6M chips don't support changing their baudrate ;_(
	// GpsUart::initialize<Board::systemClock, 115'200>();
	// configurePort();

	configureUpdateRate();
	GpsUart::discardReceiveBuffer();

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
	msg.field_baudRate().setScaled(115'200);
	MODM_LOG_INFO << "GPS baudrate set to " << msg.field_baudRate().value() << modm::endl;

	sendMessage(msg);
}

void
GpsTask::configureUpdateRate()
{
	using OutCfgRate = ublox::message::CfgRate<OutMessage>;
	OutCfgRate msg;

	msg.field_measRate().setScaled(200);
	MODM_LOG_INFO << "GPS measurement rate set to " << msg.field_measRate().value() << modm::endl;

	sendMessage(msg);
}

void
GpsTask::handle(InNavPosllh& msg)
{
	MODM_LOG_INFO << "POS: lat=" << comms::units::getDegrees<double>(msg.field_lat()) <<
		"; lon=" << comms::units::getDegrees<double>(msg.field_lon()) <<
		"; alt=" << comms::units::getMeters<double>(msg.field_height()) << modm::endl;
}

void
GpsTask::handle(InMessage&)
{}

void
GpsTask::performRead()
{
	if (uint8_t data; Usart1::read(data))
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

		if (timer.execute())
		{
			using OutNavPosllhPoll = ublox::message::NavPosllhPoll<OutMessage>;
			sendMessage(OutNavPosllhPoll());
		}

		PT_YIELD();
	}

	PT_END();
}

}