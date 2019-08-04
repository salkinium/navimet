/// @file
/// @brief Contains definition of <b>"fTOW"</b> field.

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

/// @brief Definition of <b>"fTOW"</b> field.
/// @tparam TOpt Protocol options.
/// @tparam TExtraOpts Extra options.
template <typename TOpt = ublox::DefaultOptions, typename... TExtraOpts>
struct Ftow : public
    comms::field::IntValue<
        ublox::field::FieldBase<>,
        std::int32_t,
        TExtraOpts...,
        comms::option::UnitsNanoseconds,
        comms::option::ValidNumValueRange<-500000L, 500000L>
    >
{
    /// @brief Name of the field.
    static const char* name()
    {
        return "fTOW";
    }
    
};

} // namespace field

} // namespace ublox


