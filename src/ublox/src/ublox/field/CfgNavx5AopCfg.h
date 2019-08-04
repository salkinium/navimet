/// @file
/// @brief Contains definition of <b>"aopCfg"</b> field.

#pragma once

#include "comms/field/BitmaskValue.h"
#include "comms/options.h"
#include "ublox/DefaultOptions.h"
#include "ublox/field/FieldBase.h"

namespace ublox
{

namespace field
{

/// @brief Definition of <b>"aopCfg"</b> field.
/// @tparam TOpt Protocol options.
/// @tparam TExtraOpts Extra options.
template <typename TOpt = ublox::DefaultOptions, typename... TExtraOpts>
class CfgNavx5AopCfg : public
    comms::field::BitmaskValue<
        ublox::field::FieldBase<>,
        TExtraOpts...,
        comms::option::FixedLength<1U>,
        comms::option::BitmaskReservedBits<0xFEU, 0x0U>
    >
{
    using Base = 
        comms::field::BitmaskValue<
            ublox::field::FieldBase<>,
            TExtraOpts...,
            comms::option::FixedLength<1U>,
            comms::option::BitmaskReservedBits<0xFEU, 0x0U>
        >;
public:
    /// @brief Provides names and generates access functions for internal bits.
    /// @details See definition of @b COMMS_BITMASK_BITS_SEQ macro
    ///     related to @b comms::field::BitmaskValue class from COMMS library
    ///     for details.
    ///
    ///      The generated enum values and functions are:
    ///      @li @b BitIdx_useAOP, @b getBitValue_useAOP() and @b setBitValue_useAOP().
    COMMS_BITMASK_BITS_SEQ(
        useAOP
    );
    
    /// @brief Name of the field.
    static const char* name()
    {
        return "aopCfg";
    }
    
};

} // namespace field

} // namespace ublox


