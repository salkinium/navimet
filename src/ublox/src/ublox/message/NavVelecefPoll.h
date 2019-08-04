/// @file
/// @brief Contains definition of <b>"NAV-VELECEF (Poll)"</b> message and its fields.

#pragma once

#include <tuple>
#include "comms/MessageBase.h"
#include "ublox/DefaultOptions.h"
#include "ublox/MsgId.h"

namespace ublox
{

namespace message
{

/// @brief Fields of @ref NavVelecefPoll.
/// @tparam TOpt Extra options
/// @see @ref NavVelecefPoll
/// @headerfile "ublox/message/NavVelecefPoll.h"
template <typename TOpt = ublox::DefaultOptions>
struct NavVelecefPollFields
{
    /// @brief All the fields bundled in std::tuple.
    using All = std::tuple<
    >;
};

/// @brief Definition of <b>"NAV-VELECEF (Poll)"</b> message class.
/// @details
///     See @ref NavVelecefPollFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/NavVelecefPoll.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class NavVelecefPoll : public
    comms::MessageBase<
        TMsgBase,
        typename TOpt::message::NavVelecefPoll,
        comms::option::StaticNumIdImpl<ublox::MsgId_NavVelecef>,
        comms::option::FieldsImpl<typename NavVelecefPollFields<TOpt>::All>,
        comms::option::MsgType<NavVelecefPoll<TMsgBase, TOpt> >,
        comms::option::HasName
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            typename TOpt::message::NavVelecefPoll,
            comms::option::StaticNumIdImpl<ublox::MsgId_NavVelecef>,
            comms::option::FieldsImpl<typename NavVelecefPollFields<TOpt>::All>,
            comms::option::MsgType<NavVelecefPoll<TMsgBase, TOpt> >,
            comms::option::HasName
        >;

public:
    // Compile time check for serialisation length.
    static const std::size_t MsgMinLen = Base::doMinLength();
    static const std::size_t MsgMaxLen = Base::doMaxLength();
    static_assert(MsgMinLen == 0U, "Unexpected min serialisation length");
    static_assert(MsgMaxLen == 0U, "Unexpected max serialisation length");
    
    /// @brief Name of the message.
    static const char* doName()
    {
        return "NAV-VELECEF (Poll)";
    }
    
    
};

} // namespace message

} // namespace ublox


