/// @file
/// @brief Contains definition of <b>"NAV-SVINFO (Poll)"</b> message and its fields.

#pragma once

#include <tuple>
#include "comms/MessageBase.h"
#include "ublox/DefaultOptions.h"
#include "ublox/MsgId.h"

namespace ublox
{

namespace message
{

/// @brief Fields of @ref NavSvinfoPoll.
/// @tparam TOpt Extra options
/// @see @ref NavSvinfoPoll
/// @headerfile "ublox/message/NavSvinfoPoll.h"
template <typename TOpt = ublox::DefaultOptions>
struct NavSvinfoPollFields
{
    /// @brief All the fields bundled in std::tuple.
    using All = std::tuple<
    >;
};

/// @brief Definition of <b>"NAV-SVINFO (Poll)"</b> message class.
/// @details
///     See @ref NavSvinfoPollFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/NavSvinfoPoll.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class NavSvinfoPoll : public
    comms::MessageBase<
        TMsgBase,
        typename TOpt::message::NavSvinfoPoll,
        comms::option::StaticNumIdImpl<ublox::MsgId_NavSvinfo>,
        comms::option::FieldsImpl<typename NavSvinfoPollFields<TOpt>::All>,
        comms::option::MsgType<NavSvinfoPoll<TMsgBase, TOpt> >,
        comms::option::HasName
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            typename TOpt::message::NavSvinfoPoll,
            comms::option::StaticNumIdImpl<ublox::MsgId_NavSvinfo>,
            comms::option::FieldsImpl<typename NavSvinfoPollFields<TOpt>::All>,
            comms::option::MsgType<NavSvinfoPoll<TMsgBase, TOpt> >,
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
        return "NAV-SVINFO (Poll)";
    }
    
    
};

} // namespace message

} // namespace ublox


