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

#ifndef NAVIMET_IMU_TASK_HPP
#define NAVIMET_IMU_TASK_HPP

#include <modm/processing/protothread.hpp>

namespace navimet
{

class ImuTask : public modm::pt::Protothread
{
public:
	void
	initialize();

	bool
	update();

	inline uint16_t
	heading() const
	{ return m_heading; }

protected:
	uint16_t m_heading{0};
};

}


#endif // NAVIMET_GPS_TASK_HPP