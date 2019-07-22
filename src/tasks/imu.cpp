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

#include "imu.hpp"
#include "../board.hpp"
#include <modm/processing/timer.hpp>
#include <modm/math/geometry/angle.hpp>
#include <modm/driver/inertial/bno055.hpp>
using namespace modm::literals;

#undef	MODM_LOG_LEVEL
#define	MODM_LOG_LEVEL modm::log::INFO

namespace
{
using Scl = Board::D5;
using Sda = Board::D4;
using Master = I2cMaster1;
// using Master = BitBangI2cMaster<Scl, Sda>;

modm::bno055::Data data;
modm::Bno055<Master> device(data, modm::bno055::addr(0));
modm::ShortPeriodicTimer timer{1000};
}


namespace navimet
{

void
ImuTask::initialize()
{
	Master::connect<Sda::Sda, Scl::Scl>();
	// Master::connect<Sda::BitBang, Scl::BitBang>();
	Master::initialize<Board::SystemClock, 400_kHz>();

	MODM_LOG_INFO << "IMU initialized." << modm::endl;
}

bool
ImuTask::update()
{
	PT_BEGIN();

	PT_WAIT_UNTIL(timer.execute());
	timer.restart(33);

	PT_CALL(device.configure());

	MODM_LOG_INFO << "IMU configured." << modm::endl;

	while(1)
	{
		PT_WAIT_UNTIL(timer.execute());

		PT_CALL(device.readData());
		{
			m_heading = modm::toRadian(data.heading());
			MODM_LOG_DEBUG.printf("%lu ms: %7.3f\n", modm::Clock::now().getTime(), m_heading);
		}

		PT_YIELD();
	}

	PT_END();
}

}
