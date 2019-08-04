/// @file
/// @brief Contains definition of <b>"AID-EPH (Poll SV)"</b> message and its fields.

#pragma once

#include <cstdint>
#include <tuple>
#include "comms/MessageBase.h"
#include "comms/field/IntValue.h"
#include "comms/options.h"
#include "ublox/DefaultOptions.h"
#include "ublox/MsgId.h"
#include "ublox/field/FieldBase.h"

namespace ublox
{

namespace message
{

/// @brief Fields of @ref AidEphPollSv.
/// @tparam TOpt Extra options
/// @see @ref AidEphPollSv
/// @headerfile "ublox/message/AidEphPollSv.h"
template <typename TOpt = ublox::DefaultOptions>
struct AidEphPollSvFields
{
    /// @brief Definition of <b>"svid"</b> field.
    struct Svid : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "svid";
        }
        
    };
    
    /// @brief All the fields bundled in std::tuple.
    using All = std::tuple<
        Svid
    >;
};

/// @brief Definition of <b>"AID-EPH (Poll SV)"</b> message class.
/// @details
///     See @ref AidEphPollSvFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/AidEphPollSv.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class AidEphPollSv : public
    comms::MessageBase<
        TMsgBase,
        typename TOpt::message::AidEphPollSv,
        comms::option::StaticNumIdImpl<ublox::MsgId_AidEph>,
        comms::option::FieldsImpl<typename AidEphPollSvFields<TOpt>::All>,
        comms::option::MsgType<AidEphPollSv<TMsgBase, TOpt> >,
        comms::option::HasName
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            typename TOpt::message::AidEphPollSv,
            comms::option::StaticNumIdImpl<ublox::MsgId_AidEph>,
            comms::option::FieldsImpl<typename AidEphPollSvFields<TOpt>::All>,
            comms::option::MsgType<AidEphPollSv<TMsgBase, TOpt> >,
            comms::option::HasName
        >;

public:
    /// @brief Allow access to internal fields.
    /// @details See definition of @b COMMS_MSG_FIELDS_ACCESS macro
    ///     related to @b comms::MessageBase class from COMMS library
    ///     for details.
    ///
    ///     The generated functions are:
    ///     @li @b field_svid() for @ref AidEphPollSvFields::Svid field.
    COMMS_MSG_FIELDS_ACCESS(
        svid
    );
    
    // Compile time check for serialisation length.
    static const std::size_t MsgMinLen = Base::doMinLength();
    static const std::size_t MsgMaxLen = Base::doMaxLength();
    static_assert(MsgMinLen == 1U, "Unexpected min serialisation length");
    static_assert(MsgMaxLen == 1U, "Unexpected max serialisation length");
    
    /// @brief Name of the message.
    static const char* doName()
    {
        return "AID-EPH (Poll SV)";
    }
    
    
};

} // namespace message

} // namespace ublox


