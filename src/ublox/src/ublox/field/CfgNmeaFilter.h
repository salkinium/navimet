/// @file
/// @brief Contains definition of <b>"cfgNmeaFilter"</b> field.

#pragma once

#include "comms/field/BitmaskValue.h"
#include "comms/options.h"
#include "ublox/DefaultOptions.h"
#include "ublox/field/FieldBase.h"

namespace ublox
{

namespace field
{

/// @brief Definition of <b>"cfgNmeaFilter"</b> field.
/// @tparam TOpt Protocol options.
/// @tparam TExtraOpts Extra options.
template <typename TOpt = ublox::DefaultOptions, typename... TExtraOpts>
class CfgNmeaFilter : public
    comms::field::BitmaskValue<
        ublox::field::FieldBase<>,
        TExtraOpts...,
        comms::option::FixedLength<1U>,
        comms::option::BitmaskReservedBits<0xC0U, 0x0U>
    >
{
    using Base = 
        comms::field::BitmaskValue<
            ublox::field::FieldBase<>,
            TExtraOpts...,
            comms::option::FixedLength<1U>,
            comms::option::BitmaskReservedBits<0xC0U, 0x0U>
        >;
public:
    /// @brief Provides names and generates access functions for internal bits.
    /// @details See definition of @b COMMS_BITMASK_BITS_SEQ macro
    ///     related to @b comms::field::BitmaskValue class from COMMS library
    ///     for details.
    ///
    ///      The generated enum values and functions are:
    ///      @li @b BitIdx_posFilt, @b getBitValue_posFilt() and @b setBitValue_posFilt().
    ///      @li @b BitIdx_mskPosFilt, @b getBitValue_mskPosFilt() and @b setBitValue_mskPosFilt().
    ///      @li @b BitIdx_timeFilt, @b getBitValue_timeFilt() and @b setBitValue_timeFilt().
    ///      @li @b BitIdx_dateFilt, @b getBitValue_dateFilt() and @b setBitValue_dateFilt().
    ///      @li @b BitIdx_gpsOnlyFilter, @b getBitValue_gpsOnlyFilter() and @b setBitValue_gpsOnlyFilter().
    ///      @li @b BitIdx_trackFilt, @b getBitValue_trackFilt() and @b setBitValue_trackFilt().
    COMMS_BITMASK_BITS_SEQ(
        posFilt,
        mskPosFilt,
        timeFilt,
        dateFilt,
        gpsOnlyFilter,
        trackFilt
    );
    
    /// @brief Name of the field.
    static const char* name()
    {
        return "cfgNmeaFilter";
    }
    
};

} // namespace field

} // namespace ublox


