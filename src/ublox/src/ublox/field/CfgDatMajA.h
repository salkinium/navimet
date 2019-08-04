/// @file
/// @brief Contains definition of <b>"cfgDatMajA"</b> field.

#pragma once

#include "comms/field/FloatValue.h"
#include "comms/options.h"
#include "ublox/DefaultOptions.h"
#include "ublox/field/FieldBase.h"

namespace ublox
{

namespace field
{

/// @brief Definition of <b>"cfgDatMajA"</b> field.
/// @tparam TOpt Protocol options.
/// @tparam TExtraOpts Extra options.
template <typename TOpt = ublox::DefaultOptions, typename... TExtraOpts>
class CfgDatMajA : public
    comms::field::FloatValue<
        ublox::field::FieldBase<>,
        double,
        TExtraOpts...,
        comms::option::UnitsMeters,
        comms::option::InvalidByDefault
    >
{
    using Base = 
        comms::field::FloatValue<
            ublox::field::FieldBase<>,
            double,
            TExtraOpts...,
            comms::option::UnitsMeters,
            comms::option::InvalidByDefault
        >;
public:
    /// @brief Name of the field.
    static const char* name()
    {
        return "cfgDatMajA";
    }
    
    /// @brief Custom validity check
    bool valid() const
    {
        if (Base::valid()) {
            return true;
        }
    
        if ((static_cast<typename Base::ValueType>(6300000.000000) <= Base::value()) &&
            (Base::value() <= static_cast<typename Base::ValueType>(6500000.000000))) {
            return true;
        }
        
        return false;
    }
    
};

} // namespace field

} // namespace ublox


