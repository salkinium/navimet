/// @file
/// @brief Contains definition of <b>"NAV-AOPSTATUS"</b> message and its fields.

#pragma once

#include <cstdint>
#include <tuple>
#include "comms/MessageBase.h"
#include "comms/field/BitmaskValue.h"
#include "comms/field/IntValue.h"
#include "comms/options.h"
#include "ublox/DefaultOptions.h"
#include "ublox/MsgId.h"
#include "ublox/field/FieldBase.h"
#include "ublox/field/Itow.h"
#include "ublox/field/Res1.h"
#include "ublox/field/Res4.h"

namespace ublox
{

namespace message
{

/// @brief Fields of @ref NavAopstatus.
/// @tparam TOpt Extra options
/// @see @ref NavAopstatus
/// @headerfile "ublox/message/NavAopstatus.h"
template <typename TOpt = ublox::DefaultOptions>
struct NavAopstatusFields
{
    /// @brief Definition of <b>"iTOW"</b> field.
    using Itow =
        ublox::field::Itow<
           TOpt
       >;
    
    /// @brief Definition of <b>"aopCfg"</b> field.
    class AopCfg : public
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
        ///      @li @b BitIdx_useAOP, @b getBitValue_useAOP() and @b setBitValue_useAOP().
        COMMS_BITMASK_BITS_SEQ(
            useAOP
        );
        
        /// @brief Name of the field.
        static const char* name()
        {
            return "aopCfg";
        }
        
    };
    
    /// @brief Definition of <b>"status"</b> field.
    struct Status : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "status";
        }
        
    };
    
    /// @brief Definition of <b>"reserved0"</b> field.
    struct Reserved0 : public
        ublox::field::Res1<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "reserved0";
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
    
    /// @brief Definition of <b>"availGPS"</b> field.
    struct AvailGPS : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint32_t
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "availGPS";
        }
        
    };
    
    /// @brief Definition of <b>"reserved2"</b> field.
    struct Reserved2 : public
        ublox::field::Res4<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "reserved2";
        }
        
    };
    
    /// @brief Definition of <b>"reserved3"</b> field.
    struct Reserved3 : public
        ublox::field::Res4<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "reserved3";
        }
        
    };
    
    /// @brief All the fields bundled in std::tuple.
    using All = std::tuple<
        Itow,
        AopCfg,
        Status,
        Reserved0,
        Reserved1,
        AvailGPS,
        Reserved2,
        Reserved3
    >;
};

/// @brief Definition of <b>"NAV-AOPSTATUS"</b> message class.
/// @details
///     See @ref NavAopstatusFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/NavAopstatus.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class NavAopstatus : public
    comms::MessageBase<
        TMsgBase,
        typename TOpt::message::NavAopstatus,
        comms::option::StaticNumIdImpl<ublox::MsgId_NavAopstatus>,
        comms::option::FieldsImpl<typename NavAopstatusFields<TOpt>::All>,
        comms::option::MsgType<NavAopstatus<TMsgBase, TOpt> >,
        comms::option::HasName
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            typename TOpt::message::NavAopstatus,
            comms::option::StaticNumIdImpl<ublox::MsgId_NavAopstatus>,
            comms::option::FieldsImpl<typename NavAopstatusFields<TOpt>::All>,
            comms::option::MsgType<NavAopstatus<TMsgBase, TOpt> >,
            comms::option::HasName
        >;

public:
    /// @brief Allow access to internal fields.
    /// @details See definition of @b COMMS_MSG_FIELDS_ACCESS macro
    ///     related to @b comms::MessageBase class from COMMS library
    ///     for details.
    ///
    ///     The generated functions are:
    ///     @li @b field_itow() for @ref NavAopstatusFields::Itow field.
    ///     @li @b field_aopCfg() for @ref NavAopstatusFields::AopCfg field.
    ///     @li @b field_status() for @ref NavAopstatusFields::Status field.
    ///     @li @b field_reserved0() for @ref NavAopstatusFields::Reserved0 field.
    ///     @li @b field_reserved1() for @ref NavAopstatusFields::Reserved1 field.
    ///     @li @b field_availGPS() for @ref NavAopstatusFields::AvailGPS field.
    ///     @li @b field_reserved2() for @ref NavAopstatusFields::Reserved2 field.
    ///     @li @b field_reserved3() for @ref NavAopstatusFields::Reserved3 field.
    COMMS_MSG_FIELDS_ACCESS(
        itow,
        aopCfg,
        status,
        reserved0,
        reserved1,
        availGPS,
        reserved2,
        reserved3
    );
    
    // Compile time check for serialisation length.
    static const std::size_t MsgMinLen = Base::doMinLength();
    static const std::size_t MsgMaxLen = Base::doMaxLength();
    static_assert(MsgMinLen == 20U, "Unexpected min serialisation length");
    static_assert(MsgMaxLen == 20U, "Unexpected max serialisation length");
    
    /// @brief Name of the message.
    static const char* doName()
    {
        return "NAV-AOPSTATUS";
    }
    
    
};

} // namespace message

} // namespace ublox


