/// @file
/// @brief Contains definition of <b>"sec"</b> field.

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

/// @brief Definition of <b>"sec"</b> field.
/// @tparam TOpt Protocol options.
/// @tparam TExtraOpts Extra options.
template <typename TOpt = ublox::DefaultOptions, typename... TExtraOpts>
struct Sec : public
    comms::field::IntValue<
        ublox::field::FieldBase<>,
        std::uint8_t,
        TExtraOpts...,
        comms::option::UnitsSeconds,
        comms::option::ValidNumValueRange<0, 60>
    >
{
    /// @brief Name of the field.
    static const char* name()
    {
        return "sec";
    }
    
};

} // namespace field

} // namespace ublox


