/// @file
/// @brief Contains definition of <b>"sbasService"</b> field.

#pragma once

#include "comms/field/BitmaskValue.h"
#include "comms/options.h"
#include "ublox/DefaultOptions.h"
#include "ublox/field/FieldBase.h"

namespace ublox
{

namespace field
{

/// @brief Definition of <b>"sbasService"</b> field.
/// @tparam TOpt Protocol options.
/// @tparam TExtraOpts Extra options.
template <typename TOpt = ublox::DefaultOptions, typename... TExtraOpts>
class SbasService : public
    comms::field::BitmaskValue<
        ublox::field::FieldBase<>,
        TExtraOpts...,
        comms::option::FixedLength<1U>,
        comms::option::BitmaskReservedBits<0xF0U, 0x0U>
    >
{
    using Base = 
        comms::field::BitmaskValue<
            ublox::field::FieldBase<>,
            TExtraOpts...,
            comms::option::FixedLength<1U>,
            comms::option::BitmaskReservedBits<0xF0U, 0x0U>
        >;
public:
    /// @brief Provides names and generates access functions for internal bits.
    /// @details See definition of @b COMMS_BITMASK_BITS_SEQ macro
    ///     related to @b comms::field::BitmaskValue class from COMMS library
    ///     for details.
    ///
    ///      The generated enum values and functions are:
    ///      @li @b BitIdx_Ranging, @b getBitValue_Ranging() and @b setBitValue_Ranging().
    ///      @li @b BitIdx_Corrections, @b getBitValue_Corrections() and @b setBitValue_Corrections().
    ///      @li @b BitIdx_Integrity, @b getBitValue_Integrity() and @b setBitValue_Integrity().
    ///      @li @b BitIdx_Testmode, @b getBitValue_Testmode() and @b setBitValue_Testmode().
    COMMS_BITMASK_BITS_SEQ(
        Ranging,
        Corrections,
        Integrity,
        Testmode
    );
    
    /// @brief Name of the field.
    static const char* name()
    {
        return "sbasService";
    }
    
};

} // namespace field

} // namespace ublox


