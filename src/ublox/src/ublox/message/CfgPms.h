/// @file
/// @brief Contains definition of <b>"CFG-PMS"</b> message and its fields.

#pragma once

#include <cstdint>
#include <tuple>
#include "comms/MessageBase.h"
#include "comms/field/EnumValue.h"
#include "comms/field/IntValue.h"
#include "comms/options.h"
#include "ublox/DefaultOptions.h"
#include "ublox/MsgId.h"
#include "ublox/field/FieldBase.h"
#include "ublox/field/Res2.h"

namespace ublox
{

namespace message
{

/// @brief Fields of @ref CfgPms.
/// @tparam TOpt Extra options
/// @see @ref CfgPms
/// @headerfile "ublox/message/CfgPms.h"
template <typename TOpt = ublox::DefaultOptions>
struct CfgPmsFields
{
    /// @brief Definition of <b>"version"</b> field.
    struct Version : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t,
            comms::option::ValidNumValue<0>
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "version";
        }
        
    };
    
    /// @brief Values enumerator for @ref PowerSetupValue field.
    enum class PowerSetupValueVal : std::uint8_t
    {
        FullPower = 0, ///< value <b>Full power</b>.
        Balanced = 1, ///< value @b Balanced
        Interval = 2, ///< value @b Interval
        Agressive1Hz = 3, ///< value <b>Aggressive with 1Hz</b>.
        Agressive2Hz = 4, ///< value <b>Aggressive with 2Hz</b>.
        Agressive4Hz = 5, ///< value <b>Aggressive with 4Hz</b>.
        Invalid = 255, ///< value @b Invalid
        
    };
    
    /// @brief Definition of <b>"powerSetupValue"</b> field.
    struct PowerSetupValue : public
        comms::field::EnumValue<
            ublox::field::FieldBase<>,
            PowerSetupValueVal,
            comms::option::ValidNumValueRange<0, 5>,
            comms::option::ValidNumValue<255>
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "powerSetupValue";
        }
        
    };
    
    /// @brief Definition of <b>"period"</b> field.
    struct Period : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint16_t,
            comms::option::UnitsSeconds
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "period";
        }
        
    };
    
    /// @brief Definition of <b>"onTime"</b> field.
    struct OnTime : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint16_t,
            comms::option::UnitsSeconds
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "onTime";
        }
        
    };
    
    /// @brief Definition of <b>"reserved1"</b> field.
    struct Reserved1 : public
        ublox::field::Res2<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "reserved1";
        }
        
    };
    
    /// @brief All the fields bundled in std::tuple.
    using All = std::tuple<
        Version,
        PowerSetupValue,
        Period,
        OnTime,
        Reserved1
    >;
};

/// @brief Definition of <b>"CFG-PMS"</b> message class.
/// @details
///     See @ref CfgPmsFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/CfgPms.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class CfgPms : public
    comms::MessageBase<
        TMsgBase,
        comms::option::StaticNumIdImpl<ublox::MsgId_CfgPms>,
        comms::option::FieldsImpl<typename CfgPmsFields<TOpt>::All>,
        comms::option::MsgType<CfgPms<TMsgBase, TOpt> >,
        comms::option::HasName
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            comms::option::StaticNumIdImpl<ublox::MsgId_CfgPms>,
            comms::option::FieldsImpl<typename CfgPmsFields<TOpt>::All>,
            comms::option::MsgType<CfgPms<TMsgBase, TOpt> >,
            comms::option::HasName
        >;

public:
    /// @brief Allow access to internal fields.
    /// @details See definition of @b COMMS_MSG_FIELDS_ACCESS macro
    ///     related to @b comms::MessageBase class from COMMS library
    ///     for details.
    ///
    ///     The generated functions are:
    ///     @li @b field_version() for @ref CfgPmsFields::Version field.
    ///     @li @b field_powerSetupValue() for @ref CfgPmsFields::PowerSetupValue field.
    ///     @li @b field_period() for @ref CfgPmsFields::Period field.
    ///     @li @b field_onTime() for @ref CfgPmsFields::OnTime field.
    ///     @li @b field_reserved1() for @ref CfgPmsFields::Reserved1 field.
    COMMS_MSG_FIELDS_ACCESS(
        version,
        powerSetupValue,
        period,
        onTime,
        reserved1
    );
    
    // Compile time check for serialisation length.
    static const std::size_t MsgMinLen = Base::doMinLength();
    static const std::size_t MsgMaxLen = Base::doMaxLength();
    static_assert(MsgMinLen == 8U, "Unexpected min serialisation length");
    static_assert(MsgMaxLen == 8U, "Unexpected max serialisation length");
    
    /// @brief Name of the message.
    static const char* doName()
    {
        return "CFG-PMS";
    }
    
    
};

} // namespace message

} // namespace ublox


