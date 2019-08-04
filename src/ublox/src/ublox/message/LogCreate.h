/// @file
/// @brief Contains definition of <b>"LOG-CREATE"</b> message and its fields.

#pragma once

#include <cstdint>
#include <tuple>
#include "comms/MessageBase.h"
#include "comms/field/BitmaskValue.h"
#include "comms/field/EnumValue.h"
#include "comms/field/IntValue.h"
#include "comms/options.h"
#include "ublox/DefaultOptions.h"
#include "ublox/MsgId.h"
#include "ublox/field/FieldBase.h"
#include "ublox/field/Res1.h"

namespace ublox
{

namespace message
{

/// @brief Fields of @ref LogCreate.
/// @tparam TOpt Extra options
/// @see @ref LogCreate
/// @headerfile "ublox/message/LogCreate.h"
template <typename TOpt = ublox::DefaultOptions>
struct LogCreateFields
{
    /// @brief Definition of <b>"version"</b> field.
    struct Version : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t,
            comms::option::ValidNumValue<0>
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "version";
        }
        
    };
    
    /// @brief Definition of <b>"logCfg"</b> field.
    class LogCfg : public
        comms::field::BitmaskValue<
            ublox::field::FieldBase<>,
            comms::option::FixedLength<1U>,
            comms::option::BitmaskReservedBits<0xFEU, 0x0U>
        >
    {
        using Base = 
            comms::field::BitmaskValue<
                ublox::field::FieldBase<>,
                comms::option::FixedLength<1U>,
                comms::option::BitmaskReservedBits<0xFEU, 0x0U>
            >;
    public:
        /// @brief Provides names and generates access functions for internal bits.
        /// @details See definition of @b COMMS_BITMASK_BITS_SEQ macro
        ///     related to @b comms::field::BitmaskValue class from COMMS library
        ///     for details.
        ///
        ///      The generated enum values and functions are:
        ///      @li @b BitIdx_logCfg, @b getBitValue_logCfg() and @b setBitValue_logCfg().
        COMMS_BITMASK_BITS_SEQ(
            logCfg
        );
        
        /// @brief Name of the field.
        static const char* name()
        {
            return "logCfg";
        }
        
    };
    
    /// @brief Definition of <b>"reserved1"</b> field.
    struct Reserved1 : public
        ublox::field::Res1<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "reserved1";
        }
        
    };
    
    /// @brief Values enumerator for @ref LogSize field.
    enum class LogSizeVal : std::uint8_t
    {
        Maximum = 0, ///< value @b Maximum
        Minimum = 1, ///< value @b Minimum
        UserDefined = 2, ///< value @b UserDefined
        
    };
    
    /// @brief Definition of <b>"logSize"</b> field.
    struct LogSize : public
        comms::field::EnumValue<
            ublox::field::FieldBase<>,
            LogSizeVal,
            comms::option::ValidNumValueRange<0, 2>
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "logSize";
        }
        
    };
    
    /// @brief Definition of <b>"userDefinedSize"</b> field.
    struct UserDefinedSize : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint32_t
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "userDefinedSize";
        }
        
    };
    
    /// @brief All the fields bundled in std::tuple.
    using All = std::tuple<
        Version,
        LogCfg,
        Reserved1,
        LogSize,
        UserDefinedSize
    >;
};

/// @brief Definition of <b>"LOG-CREATE"</b> message class.
/// @details
///     See @ref LogCreateFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/LogCreate.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class LogCreate : public
    comms::MessageBase<
        TMsgBase,
        typename TOpt::message::LogCreate,
        comms::option::StaticNumIdImpl<ublox::MsgId_LogCreate>,
        comms::option::FieldsImpl<typename LogCreateFields<TOpt>::All>,
        comms::option::MsgType<LogCreate<TMsgBase, TOpt> >,
        comms::option::HasName
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            typename TOpt::message::LogCreate,
            comms::option::StaticNumIdImpl<ublox::MsgId_LogCreate>,
            comms::option::FieldsImpl<typename LogCreateFields<TOpt>::All>,
            comms::option::MsgType<LogCreate<TMsgBase, TOpt> >,
            comms::option::HasName
        >;

public:
    /// @brief Allow access to internal fields.
    /// @details See definition of @b COMMS_MSG_FIELDS_ACCESS macro
    ///     related to @b comms::MessageBase class from COMMS library
    ///     for details.
    ///
    ///     The generated functions are:
    ///     @li @b field_version() for @ref LogCreateFields::Version field.
    ///     @li @b field_logCfg() for @ref LogCreateFields::LogCfg field.
    ///     @li @b field_reserved1() for @ref LogCreateFields::Reserved1 field.
    ///     @li @b field_logSize() for @ref LogCreateFields::LogSize field.
    ///     @li @b field_userDefinedSize() for @ref LogCreateFields::UserDefinedSize field.
    COMMS_MSG_FIELDS_ACCESS(
        version,
        logCfg,
        reserved1,
        logSize,
        userDefinedSize
    );
    
    // Compile time check for serialisation length.
    static const std::size_t MsgMinLen = Base::doMinLength();
    static const std::size_t MsgMaxLen = Base::doMaxLength();
    static_assert(MsgMinLen == 8U, "Unexpected min serialisation length");
    static_assert(MsgMaxLen == 8U, "Unexpected max serialisation length");
    
    /// @brief Name of the message.
    static const char* doName()
    {
        return "LOG-CREATE";
    }
    
    
};

} // namespace message

} // namespace ublox


