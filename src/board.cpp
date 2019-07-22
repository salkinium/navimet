/*
 * Copyright (c) 2016-2017, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "board.hpp"


Board::LoggerDevice loggerDevice;

// Set all four logger streams to use the UART
modm::log::Logger modm::log::debug(loggerDevice);
modm::log::Logger modm::log::info(loggerDevice);
modm::log::Logger modm::log::warning(loggerDevice);
modm::log::Logger modm::log::error(loggerDevice);

modm_extern_c void
modm_abandon(const char * module,
			 const char * location,
			 const char * failure,
			 uintptr_t context)
{
	MODM_LOG_ERROR << "Assertion '" << module << "." << location << "." << failure << "'";
	if (context) { MODM_LOG_ERROR << " @ " << (void *) context << " (" << (uint32_t) context << ")"; }
	MODM_LOG_ERROR << " failed! Abandoning..." << modm::endl;

	Board::Leds::setOutput();
	for(int times=10; times>=0; times--)
	{
		Board::Leds::write(1);
		modm::delayMilliseconds(20);
		Board::Leds::write(0);
		modm::delayMilliseconds(180);
	}
}
