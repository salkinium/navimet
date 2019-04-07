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

#include "ring.hpp"
#include <modm/board.hpp>
#include <modm/processing/timer.hpp>
#include <modm/driver/pwm/ws2812b.hpp>
#include <modm/ui/led/tables.hpp>
#include <modm/architecture/interface/atomic_lock.hpp>
#include <tuple>

#undef	MODM_LOG_LEVEL
#define	MODM_LOG_LEVEL modm::log::INFO

using Output = Board::D11;
constexpr size_t NumberOfLeds = 24;
modm::Ws2812b<SpiMaster1, Output, NumberOfLeds> leds;

float m_heading = -1;
float m_relative_heading = -1;
modm::ShortPeriodicTimer tmr{33};

std::tuple<uint8_t, uint8_t, uint8_t, uint8_t>
mapHeading(float heading)
{
	const float index = (heading * NumberOfLeds) / 360.f;
	const float lower = floorf(index);
	const float upper = lower + 1;
	const uint8_t iupper = modm::ui::table22_8_256[(index - lower) * 255];
	const uint8_t ilower = modm::ui::table22_8_256[(upper - index) * 255];

	return {uint8_t(lower) % NumberOfLeds, ilower,
			uint8_t(upper) % NumberOfLeds, iupper};
}

namespace navimet
{

void
RingTask::initialize()
{
	leds.initialize<Board::SystemClock>();
	MODM_LOG_INFO << "Ring initialized." << modm::endl;
}

void
RingTask::setAbsoluteHeading(float heading)
{
	m_heading = (heading < 0 or heading > 360) ? -1 : heading;
}

void
RingTask::setRelativeHeading(float heading)
{
	m_relative_heading = (heading < 0 or heading > 360) ? -1 : heading;
}

bool
RingTask::update()
{
	PT_BEGIN();

	while (true)
	{
		PT_WAIT_UNTIL(tmr.execute()) ;

		leds.clear();
		if (m_heading >= 0)
		{
			const auto [l, li, u, ui] = mapHeading(m_heading);

			leds.setColor(l, li, 0, 0);
			leds.setColor(u, ui, 0, 0);
		}
		if (m_relative_heading >= 0)
		{
			const auto [l, li, u, ui] = mapHeading(m_relative_heading);

			const auto [lr, lg, lb] = leds.getColor(l);
			const auto [ur, ug, ub] = leds.getColor(u);
			leds.setColor(l, lr, lg, li);
			leds.setColor(u, ur, ug, ui);
		}

		{
			// blocking for ~600µs
			modm::atomic::Lock l;
			leds.write();
		}

	}

	PT_END();
}

}
