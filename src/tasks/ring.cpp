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
#include "../board.hpp"
#include <modm/processing/timer.hpp>
#include <modm/driver/pwm/ws2812b.hpp>
#include <modm/ui/led/tables.hpp>
#include <modm/architecture/interface/atomic_lock.hpp>
#include <modm/ui/color.hpp>
#include <modm/math.hpp>
#include <tuple>

#undef	MODM_LOG_LEVEL
#define	MODM_LOG_LEVEL modm::log::INFO

namespace
{
using Output = Board::D11;
constexpr size_t NumberOfLeds = 24;
modm::Ws2812b<SpiMaster3, Output, NumberOfLeds> output;
modm::color::Rgb leds[NumberOfLeds];

float m_heading = -1;
float m_distance = -1;
modm::ShortPeriodicTimer tmr{50};

/*
std::tuple<uint8_t, uint8_t, uint8_t, uint8_t>
mapHeading(float heading)
{
	const float index = (heading * NumberOfLeds) / (2*M_PI);
	const float lower = floorf(index);
	const float upper = lower + 1;
	const uint8_t iupper = modm::ui::table22_8_256[(index - lower) * 255];
	const uint8_t ilower = modm::ui::table22_8_256[(upper - index) * 255];

	return {uint8_t(lower) % NumberOfLeds, ilower,
			uint8_t(upper) % NumberOfLeds, iupper};
}
*/
}

namespace navimet
{

void
RingTask::initialize()
{
	output.initialize<Board::SystemClock>();
	MODM_LOG_INFO << "Ring initialized." << modm::endl;
}

void
RingTask::setHeading(float heading)
{
	m_heading = std::fmod(-heading + 4*M_PI - modm::toRadian(80), 2*M_PI);
}

void
RingTask::setDistance(float distance)
{
	m_distance = std::fabs(distance);
}

bool
RingTask::update()
{
	PT_BEGIN();

	while (true)
	{
		PT_WAIT_UNTIL(tmr.execute()) ;

		// clear LED array
		for (auto &led : leds)
			led = modm::color::Rgb();

		{
			const int8_t index = (m_heading * NumberOfLeds) / (2*M_PI);
			int8_t width = 1;
			if (m_distance < 10) width = NumberOfLeds/2;
			else if (m_distance < 210) width = (1 - ((m_distance - 10) / 200)) * NumberOfLeds/2;

			for (int8_t ii = index - width/2; ii <= index + width/2; ii++)
				leds[(ii + NumberOfLeds) % NumberOfLeds].red = 150;
		}

		// Copy LED array into output
		for (size_t ii=0; ii < NumberOfLeds; ii++)
			output.setColor(ii, leds[ii]);

		{
			// blocking for ~600µs
			modm::atomic::Lock l;
			output.write();
		}

	}

	PT_END();
}

}
