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

#include "alive.hpp"
#include <modm/board.hpp>

#undef	MODM_LOG_LEVEL
#define	MODM_LOG_LEVEL modm::log::INFO

namespace navimet
{

void
AliveTask::initialize()
{
	Board::LedD13::setOutput(modm::Gpio::Low);
	MODM_LOG_INFO << "Alive task initialized." << modm::endl;
}

bool
AliveTask::update()
{
	PT_BEGIN();

	while (true)
	{
		Board::LedD13::reset();

		PT_WAIT_UNTIL(timeout.isExpired());
		timeout.restart(100);

		Board::LedD13::set();

		PT_WAIT_UNTIL(timeout.isExpired()) ;
		timeout.restart(900);

		MODM_LOG_DEBUG << "Seconds since reboot: " << ++uptime << modm::endl;
	}

	PT_END();
}

}