/// @file
/// @brief Contains definition of <b>"UPD-SOS (Create)"</b> message and its fields.

#pragma once

#include <cstdint>
#include <tuple>
#include "comms/MessageBase.h"
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

/// @brief Fields of @ref UpdSosCreate.
/// @tparam TOpt Extra options
/// @see @ref UpdSosCreate
/// @headerfile "ublox/message/UpdSosCreate.h"
template <typename TOpt = ublox::DefaultOptions>
struct UpdSosCreateFields
{
    /// @brief Definition of <b>"cmd"</b> field.
    struct Cmd : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t,
            comms::option::FailOnInvalid<>,
            comms::option::ValidNumValue<0>
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "cmd";
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
    
    /// @brief All the fields bundled in std::tuple.
    using All = std::tuple<
        Cmd,
        Reserved1
    >;
};

/// @brief Definition of <b>"UPD-SOS (Create)"</b> message class.
/// @details
///     See @ref UpdSosCreateFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/UpdSosCreate.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class UpdSosCreate : public
    comms::MessageBase<
        TMsgBase,
        typename TOpt::message::UpdSosCreate,
        comms::option::StaticNumIdImpl<ublox::MsgId_UpdSos>,
        comms::option::FieldsImpl<typename UpdSosCreateFields<TOpt>::All>,
        comms::option::MsgType<UpdSosCreate<TMsgBase, TOpt> >,
        comms::option::HasName
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            typename TOpt::message::UpdSosCreate,
            comms::option::StaticNumIdImpl<ublox::MsgId_UpdSos>,
            comms::option::FieldsImpl<typename UpdSosCreateFields<TOpt>::All>,
            comms::option::MsgType<UpdSosCreate<TMsgBase, TOpt> >,
            comms::option::HasName
        >;

public:
    /// @brief Allow access to internal fields.
    /// @details See definition of @b COMMS_MSG_FIELDS_ACCESS macro
    ///     related to @b comms::MessageBase class from COMMS library
    ///     for details.
    ///
    ///     The generated functions are:
    ///     @li @b field_cmd() for @ref UpdSosCreateFields::Cmd field.
    ///     @li @b field_reserved1() for @ref UpdSosCreateFields::Reserved1 field.
    COMMS_MSG_FIELDS_ACCESS(
        cmd,
        reserved1
    );
    
    // Compile time check for serialisation length.
    static const std::size_t MsgMinLen = Base::doMinLength();
    static const std::size_t MsgMaxLen = Base::doMaxLength();
    static_assert(MsgMinLen == 4U, "Unexpected min serialisation length");
    static_assert(MsgMaxLen == 4U, "Unexpected max serialisation length");
    
    /// @brief Name of the message.
    static const char* doName()
    {
        return "UPD-SOS (Create)";
    }
    
    
};

} // namespace message

} // namespace ublox


