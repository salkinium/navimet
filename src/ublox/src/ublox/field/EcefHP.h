/// @file
/// @brief Contains definition of <b>"ecefHP"</b> field.

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

/// @brief Definition of <b>"ecefHP"</b> field.
/// @tparam TOpt Protocol options.
/// @tparam TExtraOpts Extra options.
template <typename TOpt = ublox::DefaultOptions, typename... TExtraOpts>
struct EcefHP : public
    comms::field::IntValue<
        ublox::field::FieldBase<>,
        std::int8_t,
        TExtraOpts...,
        comms::option::ScalingRatio<1, 10>,
        comms::option::UnitsMillimeters,
        comms::option::ValidNumValueRange<-99, 99>
    >
{
    /// @brief Name of the field.
    static const char* name()
    {
        return "ecefHP";
    }
    
};

} // namespace field

} // namespace ublox


