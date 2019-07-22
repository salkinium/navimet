/*
 * Copyright (c) 2019, Niklas Hauser
 *
 * This file is part of the Navimet project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once
#include <modm/processing/protothread.hpp>

namespace navimet
{

class CommTask : public modm::pt::Protothread
{
public:
	void
	initialize();

	void
	sendPosition(float lat, float lon);

	bool
	receivePosition(float *lat, float *lon);

	bool
	update();
};

}
