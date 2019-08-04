//
// Copyright 2014 - 2016 (C). Alex Robenko. All rights reserved.
//

// This library is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/// @file comms/traits.h
/// This file contains all the classes necessary to properly
/// define message traits.


#pragma once

#include <ratio>
#include "util/access.h"

namespace comms
{

namespace traits
{

namespace endian
{

/// @brief Empty class used in traits to indicate Big Endian.
using Big = util::traits::endian::Big;

/// @brief Empty class used in traits to indicate Little Endian.
using Little = util::traits::endian::Little;

}  // namespace endian

namespace units
{

/// @brief Tag class used to indicate time value
struct Time {};

/// @brief Tag class used to indicate distance value
struct Distance {};

/// @brief Tag class used to indicate speed value
struct Speed {};

/// @brief Tag class used to indicate frequency value
struct Frequency {};

/// @brief Tag class used to indicate angle value
struct Angle {};

/// @brief Tag class used to indicate electrical current value
struct Current {};

/// @brief Tag class used to indicate electrical voltage value
struct Voltage {};

using NanosecondsRatio = std::nano;
using MicrosecondsRatio = std::micro;
using MillisecondsRatio = std::milli;
using SecondsRatio = std::ratio<1, 1>;
using MinutesRatio = std::ratio<60>;
using HoursRatio = std::ratio<60 * 60>;
using DaysRatio = std::ratio<24L * 60 * 60>;
using WeeksRatio = std::ratio<7L * 24 * 60 * 60>;

using NanometersRatio = std::nano;
using MicrometersRatio = std::micro;
using MillimetersRatio = std::milli;
using CentimetersRatio = std::centi;
using MetersRatio = std::ratio<1, 1>;
using KilometersRatio = std::kilo;

using NanometersPerSecondRatio =
    typename std::ratio_divide<NanometersRatio, SecondsRatio>::type;

using MicrometersPerSecondRatio =
    typename std::ratio_divide<MicrometersRatio, SecondsRatio>::type;

using MillimetersPerSecondRatio =
    typename std::ratio_divide<MillimetersRatio, SecondsRatio>::type;

using CentimetersPerSecondRatio =
    typename std::ratio_divide<CentimetersRatio, SecondsRatio>::type;

using MetersPerSecondRatio =
    typename std::ratio_divide<MetersRatio, SecondsRatio>::type;

using KilometersPerSecondRatio =
    typename std::ratio_divide<KilometersRatio, SecondsRatio>::type;

using KilometersPerHourRatio =
    typename std::ratio_divide<KilometersRatio, HoursRatio>::type;


using HzRatio = std::ratio<1, 1>;
using KiloHzRatio = std::kilo;
using MegaHzRatio = std::mega;
using GigaHzRatio = std::giga;

using DegreesRatio = std::ratio<1, 1>;
using RadiansRatio = std::ratio<180, 1>;

using NanoampsRatio = std::nano;
using MicroampsRatio = std::micro;
using MilliampsRatio = std::milli;
using AmpsRatio = std::ratio<1, 1>;
using KiloampsRatio = std::kilo;

using NanovoltsRatio = std::nano;
using MicrovoltsRatio = std::micro;
using MillivoltsRatio = std::milli;
using VoltsRatio = std::ratio<1, 1>;
using KilovoltsRatio = std::kilo;

} // namespace units

}  // namespace traits

}  // namespace comms

