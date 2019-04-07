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

namespace navimet
{

AliveTask aliveTask;
GpsTask gpsTask;
ImuTask imuTask;
RingTask ringTask;
modm::PeriodicTimer tmr{1000};


void
Hardware::initialize()
{
	Board::initialize();
	aliveTask.initialize();
	gpsTask.initialize();
	imuTask.initialize();
	ringTask.initialize();
}

void
Hardware::update()
{
	aliveTask.update();
	gpsTask.update();
	imuTask.update();
	ringTask.update();

	ringTask.setAbsoluteHeading(imuTask.heading());

	if (tmr.execute())
	{
		MODM_LOG_DEBUG << (int)imuTask.heading() << modm::endl;
	}
}

}
