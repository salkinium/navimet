/// @file
/// @brief Contains definition of <b>"iTOW"</b> field.

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

/// @brief Definition of <b>"iTOW"</b> field.
/// @tparam TOpt Protocol options.
/// @tparam TExtraOpts Extra options.
template <typename TOpt = ublox::DefaultOptions, typename... TExtraOpts>
struct Itow : public
    comms::field::IntValue<
        ublox::field::FieldBase<>,
        std::uint32_t,
        TExtraOpts...,
        comms::option::UnitsMilliseconds,
        comms::option::ValidNumValueRange<0, 604799999L>
    >
{
    /// @brief Name of the field.
    static const char* name()
    {
        return "iTOW";
    }
    
};

} // namespace field

} // namespace ublox


