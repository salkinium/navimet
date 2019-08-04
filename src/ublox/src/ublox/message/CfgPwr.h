/// @file
/// @brief Contains definition of <b>"CFG-PWR"</b> message and its fields.

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
#include "ublox/field/Res3.h"

namespace ublox
{

namespace message
{

/// @brief Fields of @ref CfgPwr.
/// @tparam TOpt Extra options
/// @see @ref CfgPwr
/// @headerfile "ublox/message/CfgPwr.h"
template <typename TOpt = ublox::DefaultOptions>
struct CfgPwrFields
{
    /// @brief Definition of <b>"version"</b> field.
    struct Version : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t,
            comms::option::DefaultNumValue<1>,
            comms::option::ValidNumValue<1>
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "version";
        }
        
    };
    
    /// @brief Definition of <b>"reserved1"</b> field.
    struct Reserved1 : public
        ublox::field::Res3<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "reserved1";
        }
        
    };
    
    /// @brief Values enumerator for @ref State field.
    enum class StateVal : std::uint32_t
    {
        Backup = 0x42434B50UL, ///< value @b Backup
        Running = 0x52554E20UL, ///< value @b Running
        Stopped = 0x53544F50UL, ///< value @b Stopped
        
    };
    
    /// @brief Definition of <b>"state"</b> field.
    struct State : public
        comms::field::EnumValue<
            ublox::field::FieldBase<>,
            StateVal,
            comms::option::DefaultNumValue<1381322272L>,
            comms::option::ValidNumValue<1111706448L>,
            comms::option::ValidNumValue<1381322272L>,
            comms::option::ValidNumValue<1398034256L>
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "state";
        }
        
    };
    
    /// @brief All the fields bundled in std::tuple.
    using All = std::tuple<
        Version,
        Reserved1,
        State
    >;
};

/// @brief Definition of <b>"CFG-PWR"</b> message class.
/// @details
///     See @ref CfgPwrFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/CfgPwr.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class CfgPwr : public
    comms::MessageBase<
        TMsgBase,
        typename TOpt::message::CfgPwr,
        comms::option::StaticNumIdImpl<ublox::MsgId_CfgPwr>,
        comms::option::FieldsImpl<typename CfgPwrFields<TOpt>::All>,
        comms::option::MsgType<CfgPwr<TMsgBase, TOpt> >,
        comms::option::HasName
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            typename TOpt::message::CfgPwr,
            comms::option::StaticNumIdImpl<ublox::MsgId_CfgPwr>,
            comms::option::FieldsImpl<typename CfgPwrFields<TOpt>::All>,
            comms::option::MsgType<CfgPwr<TMsgBase, TOpt> >,
            comms::option::HasName
        >;

public:
    /// @brief Allow access to internal fields.
    /// @details See definition of @b COMMS_MSG_FIELDS_ACCESS macro
    ///     related to @b comms::MessageBase class from COMMS library
    ///     for details.
    ///
    ///     The generated functions are:
    ///     @li @b field_version() for @ref CfgPwrFields::Version field.
    ///     @li @b field_reserved1() for @ref CfgPwrFields::Reserved1 field.
    ///     @li @b field_state() for @ref CfgPwrFields::State field.
    COMMS_MSG_FIELDS_ACCESS(
        version,
        reserved1,
        state
    );
    
    // Compile time check for serialisation length.
    static const std::size_t MsgMinLen = Base::doMinLength();
    static const std::size_t MsgMaxLen = Base::doMaxLength();
    static_assert(MsgMinLen == 8U, "Unexpected min serialisation length");
    static_assert(MsgMaxLen == 8U, "Unexpected max serialisation length");
    
    /// @brief Name of the message.
    static const char* doName()
    {
        return "CFG-PWR";
    }
    
    
};

} // namespace message

} // namespace ublox


