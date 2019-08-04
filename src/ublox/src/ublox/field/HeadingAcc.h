/// @file
/// @brief Contains definition of <b>"headingAcc"</b> field.

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

/// @brief Definition of <b>"headingAcc"</b> field.
/// @tparam TOpt Protocol options.
/// @tparam TExtraOpts Extra options.
template <typename TOpt = ublox::DefaultOptions, typename... TExtraOpts>
struct HeadingAcc : public
    comms::field::IntValue<
        ublox::field::FieldBase<>,
        std::uint32_t,
        TExtraOpts...,
        comms::option::ScalingRatio<1, 100000L>,
        comms::option::UnitsDegrees
    >
{
    /// @brief Name of the field.
    static const char* name()
    {
        return "headingAcc";
    }
    
};

} // namespace field

} // namespace ublox


