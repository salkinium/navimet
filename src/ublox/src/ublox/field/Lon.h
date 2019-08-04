/// @file
/// @brief Contains definition of <b>"lon"</b> field.

#pragma once

#include <cstdint>
#include "comms/field/IntValue.h"
#include "comms/options.h"
#include "ublox/DefaultOptions.h"
#include "ublox/field/FieldBase.h"

namespace ublox
{

namespace field
{

/// @brief Definition of <b>"lon"</b> field.
/// @details
///     Longitude
/// @tparam TOpt Protocol options.
/// @tparam TExtraOpts Extra options.
template <typename TOpt = ublox::DefaultOptions, typename... TExtraOpts>
struct Lon : public
    comms::field::IntValue<
        ublox::field::FieldBase<>,
        std::int32_t,
        TExtraOpts...,
        comms::option::ScalingRatio<1, 10000000L>,
        comms::option::UnitsDegrees
    >
{
    /// @brief Name of the field.
    static const char* name()
    {
        return "lon";
    }
    
};

} // namespace field

} // namespace ublox


