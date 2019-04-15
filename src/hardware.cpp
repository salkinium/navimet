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

#include "hardware.hpp"
#include "tasks/alive.hpp"
#include "tasks/gps.hpp"
#include "tasks/imu.hpp"
#include "tasks/ring.hpp"
#include <modm/math.hpp>

extern modm::log::Logger modm::log::info;

namespace
{
using namespace navimet;

AliveTask aliveTask;
GpsTask gpsTask;
ImuTask imuTask;
RingTask ringTask;

// 50.779640192 6.063100336 ~ ICT Cubes
float stored_latitude{50.779640192 * M_PI / 180};
float stored_longitude{6.063100336 * M_PI / 180};
void storeCurrentLocation() {
	stored_latitude = gpsTask.latitude();
	stored_longitude = gpsTask.longitude();
	MODM_LOG_DEBUG.printf("Storing: %3.3f %3.3f\n", stored_latitude*180/M_PI, stored_longitude*180/M_PI);
}

using StoreButton = GpioInverted<Board::D9>;
}

namespace navimet
{

void
Hardware::initialize()
{
	Board::initialize();
	MODM_LOG_INFO << "REBOOT!" << modm::endl;
	aliveTask.initialize();
	gpsTask.initialize();
	imuTask.initialize();
	ringTask.initialize();

	StoreButton::setInput(StoreButton::InputType::PullUp);
}

void
Hardware::update()
{
	aliveTask.update();
	gpsTask.update();
	imuTask.update();
	ringTask.update();

	static modm::PeriodicTimer tmr{100};
	if (tmr.execute())
	{
		const float distance = gpsTask.distance_to(stored_latitude, stored_longitude);
		const float heading = gpsTask.heading_to(stored_latitude, stored_longitude);
		const float own_heading = imuTask.heading();
		ringTask.setHeading(own_heading - heading);
		ringTask.setDistance(distance);
	}

	static bool stored_button{false};
	if (bool value = StoreButton::read(); value != stored_button)
	{
		if (value) storeCurrentLocation();
		stored_button = value;
	}

	if (char command = ({char v; modm::log::info.get(v); v;}); command != modm::IOStream::eof)
	{
		switch (command) {
			case 's':
				storeCurrentLocation();
				break;
			case 'd':
				MODM_LOG_DEBUG.printf("Location: %f %f\n", gpsTask.latitude(), gpsTask.longitude());
			default:
				break;
		}
	}

	static modm::PeriodicTimer dbgtmr{1000};
	if (dbgtmr.execute())
	{
		const float distance = gpsTask.distance_to(stored_latitude, stored_longitude);
		const float heading = gpsTask.heading_to(stored_latitude, stored_longitude);
		MODM_LOG_DEBUG.printf("True Heading: %u Heading: %u Distance: %um\n", (int)modm::toDegree(imuTask.heading()), (int)modm::toDegree(heading), (int)distance);
	}
}

}
