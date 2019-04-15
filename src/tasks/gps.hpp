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

#pragma once
// STM32 headers define the RTC module as a macro
#pragma push_macro("RTC")
#pragma push_macro("USB")
#undef RTC
#undef USB
#include <ublox/Message.h>
#include <ublox/message/NavPosllh.h>
#include <ublox/frame/UbloxFrame.h>
#pragma pop_macro("RTC")
#pragma pop_macro("USB")

#include <modm/processing/protothread.hpp>
#include <string>
#include <vector>

namespace navimet
{

class GpsTask : public modm::pt::Protothread
{
public:
	using InMessage =
	    ublox::Message<
	        comms::option::ReadIterator<const std::uint8_t*>,
	        comms::option::Handler<navimet::GpsTask> // Dispatch to this object
	    >;

	using OutBuffer = std::vector<std::uint8_t>;
	using OutMessage =
	    ublox::Message<
	        comms::option::IdInfoInterface,
	        comms::option::WriteIterator<std::back_insert_iterator<OutBuffer> >,
	        comms::option::LengthInfoInterface
	    >;

	using InNavPosllh = ublox::message::NavPosllh<InMessage>;

public:
	void
	initialize();

	bool
	update();

public:
	float latitude() const;
	float longitude() const;
	float accuracy() const;

	float distance_to(float latitude, float longitude) const;
	float heading_to(float latitude, float longitude) const;

public:
    void
    handle(InNavPosllh& msg);

    void
    handle(InMessage& msg);

protected:
	void
	sendMessage(const OutMessage& msg);

	void
	configurePort();

	void
	configureUpdateRate();

	void
	performRead();

	void
	processInputData();
};

}
