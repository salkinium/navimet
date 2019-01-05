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

namespace navimet
{

AliveTask aliveTask;
GpsTask gpsTask;
ImuTask imuTask;


void
Hardware::initialize()
{
	Board::initialize();
	aliveTask.initialize();
	gpsTask.initialize();
	imuTask.initialize();
}

void
Hardware::update()
{
	aliveTask.update();
	gpsTask.update();
	imuTask.update();
}

}