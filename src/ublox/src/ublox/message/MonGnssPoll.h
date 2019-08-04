/// @file
/// @brief Contains definition of <b>"MON-GNSS (Poll)"</b> message and its fields.

#pragma once

#include <tuple>
#include "comms/MessageBase.h"
#include "ublox/DefaultOptions.h"
#include "ublox/MsgId.h"

namespace ublox
{

namespace message
{

/// @brief Fields of @ref MonGnssPoll.
/// @tparam TOpt Extra options
/// @see @ref MonGnssPoll
/// @headerfile "ublox/message/MonGnssPoll.h"
template <typename TOpt = ublox::DefaultOptions>
struct MonGnssPollFields
{
    /// @brief All the fields bundled in std::tuple.
    using All = std::tuple<
    >;
};

/// @brief Definition of <b>"MON-GNSS (Poll)"</b> message class.
/// @details
///     See @ref MonGnssPollFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/MonGnssPoll.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class MonGnssPoll : public
    comms::MessageBase<
        TMsgBase,
        typename TOpt::message::MonGnssPoll,
        comms::option::StaticNumIdImpl<ublox::MsgId_MonGnss>,
        comms::option::FieldsImpl<typename MonGnssPollFields<TOpt>::All>,
        comms::option::MsgType<MonGnssPoll<TMsgBase, TOpt> >,
        comms::option::HasName
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            typename TOpt::message::MonGnssPoll,
            comms::option::StaticNumIdImpl<ublox::MsgId_MonGnss>,
            comms::option::FieldsImpl<typename MonGnssPollFields<TOpt>::All>,
            comms::option::MsgType<MonGnssPoll<TMsgBase, TOpt> >,
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
        return "MON-GNSS (Poll)";
    }
    
    
};

} // namespace message

} // namespace ublox


