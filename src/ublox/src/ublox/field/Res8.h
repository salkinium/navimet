/// @file
/// @brief Contains definition of <b>"res8"</b> field.

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

/// @brief Definition of <b>"res8"</b> field.
/// @details
///     Reserved field with 8 bytes length
/// @tparam TOpt Protocol options.
/// @tparam TExtraOpts Extra options.
template <typename TOpt = ublox::DefaultOptions, typename... TExtraOpts>
struct Res8 : public
    comms::field::IntValue<
        ublox::field::FieldBase<>,
        std::uint64_t,
        TExtraOpts...,
        comms::option::ValidNumValue<0>
    >
{
    /// @brief Name of the field.
    static const char* name()
    {
        return "res8";
    }
    
};

} // namespace field

} // namespace ublox


