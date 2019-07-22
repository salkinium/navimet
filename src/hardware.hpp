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

#ifndef NAVIMET_HARDWARE_HPP
#define NAVIMET_HARDWARE_HPP

#include "board.hpp"
#include <modm/debug/logger.hpp>

namespace navimet::Hardware
{

void
initialize();

void
update();

}


#endif // NAVIMET_HARDWARE_HPP
