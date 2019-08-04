/// @file
/// @brief Contains definition of <b>"UPD-SOS (Ack)"</b> message and its fields.

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

/// @brief Fields of @ref UpdSosAck.
/// @tparam TOpt Extra options
/// @see @ref UpdSosAck
/// @headerfile "ublox/message/UpdSosAck.h"
template <typename TOpt = ublox::DefaultOptions>
struct UpdSosAckFields
{
    /// @brief Definition of <b>"cmd"</b> field.
    struct Cmd : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t,
            comms::option::FailOnInvalid<>,
            comms::option::DefaultNumValue<2>,
            comms::option::ValidNumValue<2>
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
    
    /// @brief Values enumerator for @ref Response field.
    enum class ResponseVal : std::uint8_t
    {
        NotAcknowledged = 0, ///< value @b NotAcknowledged
        Acknowledged = 1, ///< value @b Acknowledged
        
    };
    
    /// @brief Definition of <b>"response"</b> field.
    struct Response : public
        comms::field::EnumValue<
            ublox::field::FieldBase<>,
            ResponseVal,
            comms::option::ValidNumValueRange<0, 1>
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "response";
        }
        
    };
    
    /// @brief Definition of <b>"reserved2"</b> field.
    struct Reserved2 : public
        ublox::field::Res3<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "reserved2";
        }
        
    };
    
    /// @brief All the fields bundled in std::tuple.
    using All = std::tuple<
        Cmd,
        Reserved1,
        Response,
        Reserved2
    >;
};

/// @brief Definition of <b>"UPD-SOS (Ack)"</b> message class.
/// @details
///     See @ref UpdSosAckFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/UpdSosAck.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class UpdSosAck : public
    comms::MessageBase<
        TMsgBase,
        typename TOpt::message::UpdSosAck,
        comms::option::StaticNumIdImpl<ublox::MsgId_UpdSos>,
        comms::option::FieldsImpl<typename UpdSosAckFields<TOpt>::All>,
        comms::option::MsgType<UpdSosAck<TMsgBase, TOpt> >,
        comms::option::HasName
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            typename TOpt::message::UpdSosAck,
            comms::option::StaticNumIdImpl<ublox::MsgId_UpdSos>,
            comms::option::FieldsImpl<typename UpdSosAckFields<TOpt>::All>,
            comms::option::MsgType<UpdSosAck<TMsgBase, TOpt> >,
            comms::option::HasName
        >;

public:
    /// @brief Allow access to internal fields.
    /// @details See definition of @b COMMS_MSG_FIELDS_ACCESS macro
    ///     related to @b comms::MessageBase class from COMMS library
    ///     for details.
    ///
    ///     The generated functions are:
    ///     @li @b field_cmd() for @ref UpdSosAckFields::Cmd field.
    ///     @li @b field_reserved1() for @ref UpdSosAckFields::Reserved1 field.
    ///     @li @b field_response() for @ref UpdSosAckFields::Response field.
    ///     @li @b field_reserved2() for @ref UpdSosAckFields::Reserved2 field.
    COMMS_MSG_FIELDS_ACCESS(
        cmd,
        reserved1,
        response,
        reserved2
    );
    
    // Compile time check for serialisation length.
    static const std::size_t MsgMinLen = Base::doMinLength();
    static const std::size_t MsgMaxLen = Base::doMaxLength();
    static_assert(MsgMinLen == 8U, "Unexpected min serialisation length");
    static_assert(MsgMaxLen == 8U, "Unexpected max serialisation length");
    
    /// @brief Name of the message.
    static const char* doName()
    {
        return "UPD-SOS (Ack)";
    }
    
    
};

} // namespace message

} // namespace ublox


