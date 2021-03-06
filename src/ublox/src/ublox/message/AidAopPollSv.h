/// @file
/// @brief Contains definition of <b>"AID-AOP (Poll SV)"</b> message and its fields.

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

/// @brief Fields of @ref AidAopPollSv.
/// @tparam TOpt Extra options
/// @see @ref AidAopPollSv
/// @headerfile "ublox/message/AidAopPollSv.h"
template <typename TOpt = ublox::DefaultOptions>
struct AidAopPollSvFields
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

/// @brief Definition of <b>"AID-AOP (Poll SV)"</b> message class.
/// @details
///     See @ref AidAopPollSvFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/AidAopPollSv.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class AidAopPollSv : public
    comms::MessageBase<
        TMsgBase,
        typename TOpt::message::AidAopPollSv,
        comms::option::StaticNumIdImpl<ublox::MsgId_AidAop>,
        comms::option::FieldsImpl<typename AidAopPollSvFields<TOpt>::All>,
        comms::option::MsgType<AidAopPollSv<TMsgBase, TOpt> >,
        comms::option::HasName
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            typename TOpt::message::AidAopPollSv,
            comms::option::StaticNumIdImpl<ublox::MsgId_AidAop>,
            comms::option::FieldsImpl<typename AidAopPollSvFields<TOpt>::All>,
            comms::option::MsgType<AidAopPollSv<TMsgBase, TOpt> >,
            comms::option::HasName
        >;

public:
    /// @brief Allow access to internal fields.
    /// @details See definition of @b COMMS_MSG_FIELDS_ACCESS macro
    ///     related to @b comms::MessageBase class from COMMS library
    ///     for details.
    ///
    ///     The generated functions are:
    ///     @li @b field_svid() for @ref AidAopPollSvFields::Svid field.
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
        return "AID-AOP (Poll SV)";
    }
    
    
};

} // namespace message

} // namespace ublox


