/// @file
/// @brief Contains definition of <b>"dop"</b> field.

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

/// @brief Definition of <b>"dop"</b> field.
/// @tparam TOpt Protocol options.
/// @tparam TExtraOpts Extra options.
template <typename TOpt = ublox::DefaultOptions, typename... TExtraOpts>
struct Dop : public
    comms::field::IntValue<
        ublox::field::FieldBase<>,
        std::uint16_t,
        TExtraOpts...,
        comms::option::ScalingRatio<1, 100>
    >
{
    /// @brief Name of the field.
    static const char* name()
    {
        return "dop";
    }
    
};

} // namespace field

} // namespace ublox


