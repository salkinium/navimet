/// @file
/// @brief Contains definition of <b>"AID-ALP (Status)"</b> message and its fields.

#pragma once

#include <cstdint>
#include <tuple>
#include "comms/MessageBase.h"
#include "comms/field/EnumValue.h"
#include "comms/options.h"
#include "ublox/DefaultOptions.h"
#include "ublox/MsgId.h"
#include "ublox/field/FieldBase.h"

namespace ublox
{

namespace message
{

/// @brief Fields of @ref AidAlpStatus.
/// @tparam TOpt Extra options
/// @see @ref AidAlpStatus
/// @headerfile "ublox/message/AidAlpStatus.h"
template <typename TOpt = ublox::DefaultOptions>
struct AidAlpStatusFields
{
    /// @brief Values enumerator for @ref Status field.
    enum class StatusVal : std::uint8_t
    {
        nak = 0, ///< value @b nak
        ack = 1, ///< value @b ack
        
    };
    
    /// @brief Definition of <b>"status"</b> field.
    struct Status : public
        comms::field::EnumValue<
            ublox::field::FieldBase<>,
            StatusVal,
            comms::option::FailOnInvalid<>,
            comms::option::ValidNumValueRange<0, 1>
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "status";
        }
        
    };
    
    /// @brief All the fields bundled in std::tuple.
    using All = std::tuple<
        Status
    >;
};

/// @brief Definition of <b>"AID-ALP (Status)"</b> message class.
/// @details
///     See @ref AidAlpStatusFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/AidAlpStatus.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class AidAlpStatus : public
    comms::MessageBase<
        TMsgBase,
        comms::option::StaticNumIdImpl<ublox::MsgId_AidAlp>,
        comms::option::FieldsImpl<typename AidAlpStatusFields<TOpt>::All>,
        comms::option::MsgType<AidAlpStatus<TMsgBase, TOpt> >,
        comms::option::HasName
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            comms::option::StaticNumIdImpl<ublox::MsgId_AidAlp>,
            comms::option::FieldsImpl<typename AidAlpStatusFields<TOpt>::All>,
            comms::option::MsgType<AidAlpStatus<TMsgBase, TOpt> >,
            comms::option::HasName
        >;

public:
    /// @brief Allow access to internal fields.
    /// @details See definition of @b COMMS_MSG_FIELDS_ACCESS macro
    ///     related to @b comms::MessageBase class from COMMS library
    ///     for details.
    ///
    ///     The generated functions are:
    ///     @li @b field_status() for @ref AidAlpStatusFields::Status field.
    COMMS_MSG_FIELDS_ACCESS(
        status
    );
    
    // Compile time check for serialisation length.
    static const std::size_t MsgMinLen = Base::doMinLength();
    static const std::size_t MsgMaxLen = Base::doMaxLength();
    static_assert(MsgMinLen == 1U, "Unexpected min serialisation length");
    static_assert(MsgMaxLen == 1U, "Unexpected max serialisation length");
    
    /// @brief Name of the message.
    static const char* doName()
    {
        return "AID-ALP (Status)";
    }
    
    
};

} // namespace message

} // namespace ublox


