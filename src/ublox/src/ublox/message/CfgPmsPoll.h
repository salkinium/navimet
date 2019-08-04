/// @file
/// @brief Contains definition of <b>"CFG-PMS (Poll)"</b> message and its fields.

#pragma once

#include <tuple>
#include "comms/MessageBase.h"
#include "ublox/DefaultOptions.h"
#include "ublox/MsgId.h"

namespace ublox
{

namespace message
{

/// @brief Fields of @ref CfgPmsPoll.
/// @tparam TOpt Extra options
/// @see @ref CfgPmsPoll
/// @headerfile "ublox/message/CfgPmsPoll.h"
template <typename TOpt = ublox::DefaultOptions>
struct CfgPmsPollFields
{
    /// @brief All the fields bundled in std::tuple.
    using All = std::tuple<
    >;
};

/// @brief Definition of <b>"CFG-PMS (Poll)"</b> message class.
/// @details
///     See @ref CfgPmsPollFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/CfgPmsPoll.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class CfgPmsPoll : public
    comms::MessageBase<
        TMsgBase,
        typename TOpt::message::CfgPmsPoll,
        comms::option::StaticNumIdImpl<ublox::MsgId_CfgPms>,
        comms::option::FieldsImpl<typename CfgPmsPollFields<TOpt>::All>,
        comms::option::MsgType<CfgPmsPoll<TMsgBase, TOpt> >,
        comms::option::HasName
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            typename TOpt::message::CfgPmsPoll,
            comms::option::StaticNumIdImpl<ublox::MsgId_CfgPms>,
            comms::option::FieldsImpl<typename CfgPmsPollFields<TOpt>::All>,
            comms::option::MsgType<CfgPmsPoll<TMsgBase, TOpt> >,
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
        return "CFG-PMS (Poll)";
    }
    
    
};

} // namespace message

} // namespace ublox


