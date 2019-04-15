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

#pragma once
#include <modm/processing/timer.hpp>
#include <modm/processing/protothread.hpp>

namespace navimet
{

class AliveTask : public modm::pt::Protothread
{
public:
	void
	initialize();

	bool
	update();
};

}
