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
#include "tasks/comm.hpp"
#include <modm/math.hpp>
#include <modm/debug.hpp>

namespace
{
using namespace navimet;

AliveTask aliveTask;
GpsTask gpsTask;
ImuTask imuTask;
RingTask ringTask;
CommTask commTask;

// 50.779640192 6.063100336 ~ ICT Cubes
float stored_latitude{modm::toRadian(50.779640192)};
float stored_longitude{modm::toRadian(6.063100336)};
void storeCurrentLocation() {
	stored_latitude = gpsTask.latitude();
	stored_longitude = gpsTask.longitude();
	MODM_LOG_INFO.printf("Sending position: %3.3f %3.3f\n",
	                     double(modm::toDegree(stored_latitude)),
	                     double(modm::toDegree(stored_longitude)));
	commTask.sendPosition(stored_latitude, stored_longitude);
}

using Button1 = GpioInverted<Board::D9>;
using Button2 = GpioInverted<Board::D10>;
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
	commTask.initialize();

	Button1::setInput(Button1::InputType::PullUp);
	Button2::setInput(Button2::InputType::PullUp);
}

void
Hardware::update()
{
	aliveTask.update();
	gpsTask.update();
	imuTask.update();
	ringTask.update();
	commTask.update();

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
	if (bool value = Button1::read(); value != stored_button)
	{
		if (value) storeCurrentLocation();
		stored_button = value;
	}
	if (commTask.receivePosition(&stored_latitude, &stored_longitude))
	{
		MODM_LOG_INFO.printf("Receiving position: %3.3f %3.3f\n",
		                     double(modm::toDegree(stored_latitude)),
		                     double(modm::toDegree(stored_longitude)));
	}

	if (char command = ({char v; modm::log::info.get(v); v;}); command != modm::IOStream::eof)
	{
		switch (command) {
			case 's':
				storeCurrentLocation();
				break;
			case 'd':
				MODM_LOG_DEBUG.printf("Location: %f %f\n",
				                      double(gpsTask.latitude()),
				                      double(gpsTask.longitude()));
			default:
				break;
		}
	}

#ifdef MODM_DEBUG_BUILD
	static modm::PeriodicTimer dbgtmr{1000};
	if (dbgtmr.execute())
	{
		const float distance = gpsTask.distance_to(stored_latitude, stored_longitude);
		const float heading = gpsTask.heading_to(stored_latitude, stored_longitude);
		MODM_LOG_DEBUG.printf("True Heading: %u Heading: %u Distance: %um\n",
		                      (int)modm::toDegree(imuTask.heading()),
		                      (int)modm::toDegree(heading), (int)distance);
	}
#endif
}

}
