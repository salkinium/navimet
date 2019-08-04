/// @file
/// @brief Contains definition of <b>"MGA-BDS-TIME_UTC"</b> message and its fields.

#pragma once

#include <cstdint>
#include <tuple>
#include "comms/MessageBase.h"
#include "comms/field/Bitfield.h"
#include "comms/field/BitmaskValue.h"
#include "comms/field/EnumValue.h"
#include "comms/field/IntValue.h"
#include "comms/options.h"
#include "ublox/DefaultOptions.h"
#include "ublox/MsgId.h"
#include "ublox/field/Day.h"
#include "ublox/field/FieldBase.h"
#include "ublox/field/Hour.h"
#include "ublox/field/Min.h"
#include "ublox/field/Month.h"
#include "ublox/field/Res1.h"
#include "ublox/field/Res2.h"
#include "ublox/field/Sec.h"
#include "ublox/field/Year.h"

namespace ublox
{

namespace message
{

/// @brief Fields of @ref MgaIniTimeUtc.
/// @tparam TOpt Extra options
/// @see @ref MgaIniTimeUtc
/// @headerfile "ublox/message/MgaIniTimeUtc.h"
template <typename TOpt = ublox::DefaultOptions>
struct MgaIniTimeUtcFields
{
    /// @brief Definition of <b>"type"</b> field.
    struct Type : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t,
            comms::option::FailOnInvalid<>,
            comms::option::DefaultNumValue<16>,
            comms::option::ValidNumValue<16>
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "type";
        }
        
    };
    
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
    
    /// @brief Scope for all the member fields of @ref Ref bitfield.
    struct RefMembers
    {
        /// @brief Values enumerator for @ref Source field.
        enum class SourceVal : std::uint8_t
        {
            None = 0, ///< value @b None
            EXTINT0 = 1, ///< value @b EXTINT0
            EXTINT1 = 2, ///< value @b EXTINT1
            
        };
        
        /// @brief Definition of <b>"source"</b> field.
        struct Source : public
            comms::field::EnumValue<
                ublox::field::FieldBase<>,
                SourceVal,
                comms::option::FixedBitLength<4U>,
                comms::option::ValidNumValueRange<0, 2>
            >
        {
            /// @brief Name of the field.
            static const char* name()
            {
                return "source";
            }
            
        };
        
        /// @brief Definition of <b>""</b> field.
        class Bits : public
            comms::field::BitmaskValue<
                ublox::field::FieldBase<>,
                comms::option::FixedBitLength<4U>,
                comms::option::BitmaskReservedBits<0xCU, 0x0U>
            >
        {
            using Base = 
                comms::field::BitmaskValue<
                    ublox::field::FieldBase<>,
                    comms::option::FixedBitLength<4U>,
                    comms::option::BitmaskReservedBits<0xCU, 0x0U>
                >;
        public:
            /// @brief Provides names and generates access functions for internal bits.
            /// @details See definition of @b COMMS_BITMASK_BITS_SEQ macro
            ///     related to @b comms::field::BitmaskValue class from COMMS library
            ///     for details.
            ///
            ///      The generated enum values and functions are:
            ///      @li @b BitIdx_fall, @b getBitValue_fall() and @b setBitValue_fall().
            ///      @li @b BitIdx_last, @b getBitValue_last() and @b setBitValue_last().
            COMMS_BITMASK_BITS_SEQ(
                fall,
                last
            );
            
            /// @brief Name of the field.
            static const char* name()
            {
                return "";
            }
            
        };
        
        /// @brief All members bundled in @b std::tuple.
        using All =
            std::tuple<
               Source,
               Bits
            >;
    };
    
    /// @brief Definition of <b>"ref"</b> field.
    class Ref : public
        comms::field::Bitfield<
            ublox::field::FieldBase<>,
            typename RefMembers::All
        >
    {
        using Base = 
            comms::field::Bitfield<
                ublox::field::FieldBase<>,
                typename RefMembers::All
            >;
    public:
        /// @brief Allow access to internal fields.
        /// @details See definition of @b COMMS_FIELD_MEMBERS_ACCESS macro
        ///     related to @b comms::field::Bitfield class from COMMS library
        ///     for details.
        ///
        ///      The generated access functions are:
        ///     @li @b field_source() - for RefMembers::Source member field.
        ///     @li @b field_bits() - for RefMembers::Bits member field.
        COMMS_FIELD_MEMBERS_ACCESS(
            source,
            bits
        );
        
        /// @brief Name of the field.
        static const char* name()
        {
            return "ref";
        }
        
    };
    
    /// @brief Definition of <b>"leapSecs"</b> field.
    struct LeapSecs : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t,
            comms::option::UnitsSeconds
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "leapSecs";
        }
        
    };
    
    /// @brief Definition of <b>"year"</b> field.
    using Year =
        ublox::field::Year<
           TOpt
       >;
    
    /// @brief Definition of <b>"month"</b> field.
    using Month =
        ublox::field::Month<
           TOpt
       >;
    
    /// @brief Definition of <b>"day"</b> field.
    using Day =
        ublox::field::Day<
           TOpt
       >;
    
    /// @brief Definition of <b>"hour"</b> field.
    using Hour =
        ublox::field::Hour<
           TOpt
       >;
    
    /// @brief Definition of <b>"minute"</b> field.
    struct Minute : public
        ublox::field::Min<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "minute";
        }
        
    };
    
    /// @brief Definition of <b>"second"</b> field.
    struct Second : public
        ublox::field::Sec<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "second";
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
    
    /// @brief Definition of <b>"ns"</b> field.
    struct Ns : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint32_t,
            comms::option::UnitsNanoseconds
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "ns";
        }
        
    };
    
    /// @brief Definition of <b>"tAccS"</b> field.
    struct TAccS : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint16_t,
            comms::option::UnitsSeconds
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "tAccS";
        }
        
    };
    
    /// @brief Definition of <b>"reserved2"</b> field.
    struct Reserved2 : public
        ublox::field::Res2<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "reserved2";
        }
        
    };
    
    /// @brief Definition of <b>"tAccNs"</b> field.
    struct TAccNs : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint32_t,
            comms::option::UnitsNanoseconds
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "tAccNs";
        }
        
    };
    
    /// @brief All the fields bundled in std::tuple.
    using All = std::tuple<
        Type,
        Version,
        Ref,
        LeapSecs,
        Year,
        Month,
        Day,
        Hour,
        Minute,
        Second,
        Reserved1,
        Ns,
        TAccS,
        Reserved2,
        TAccNs
    >;
};

/// @brief Definition of <b>"MGA-BDS-TIME_UTC"</b> message class.
/// @details
///     See @ref MgaIniTimeUtcFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/MgaIniTimeUtc.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class MgaIniTimeUtc : public
    comms::MessageBase<
        TMsgBase,
        typename TOpt::message::MgaIniTimeUtc,
        comms::option::StaticNumIdImpl<ublox::MsgId_MgaIni>,
        comms::option::FieldsImpl<typename MgaIniTimeUtcFields<TOpt>::All>,
        comms::option::MsgType<MgaIniTimeUtc<TMsgBase, TOpt> >,
        comms::option::HasName
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            typename TOpt::message::MgaIniTimeUtc,
            comms::option::StaticNumIdImpl<ublox::MsgId_MgaIni>,
            comms::option::FieldsImpl<typename MgaIniTimeUtcFields<TOpt>::All>,
            comms::option::MsgType<MgaIniTimeUtc<TMsgBase, TOpt> >,
            comms::option::HasName
        >;

public:
    /// @brief Allow access to internal fields.
    /// @details See definition of @b COMMS_MSG_FIELDS_ACCESS macro
    ///     related to @b comms::MessageBase class from COMMS library
    ///     for details.
    ///
    ///     The generated functions are:
    ///     @li @b field_type() for @ref MgaIniTimeUtcFields::Type field.
    ///     @li @b field_version() for @ref MgaIniTimeUtcFields::Version field.
    ///     @li @b field_ref() for @ref MgaIniTimeUtcFields::Ref field.
    ///     @li @b field_leapSecs() for @ref MgaIniTimeUtcFields::LeapSecs field.
    ///     @li @b field_year() for @ref MgaIniTimeUtcFields::Year field.
    ///     @li @b field_month() for @ref MgaIniTimeUtcFields::Month field.
    ///     @li @b field_day() for @ref MgaIniTimeUtcFields::Day field.
    ///     @li @b field_hour() for @ref MgaIniTimeUtcFields::Hour field.
    ///     @li @b field_minute() for @ref MgaIniTimeUtcFields::Minute field.
    ///     @li @b field_second() for @ref MgaIniTimeUtcFields::Second field.
    ///     @li @b field_reserved1() for @ref MgaIniTimeUtcFields::Reserved1 field.
    ///     @li @b field_ns() for @ref MgaIniTimeUtcFields::Ns field.
    ///     @li @b field_tAccS() for @ref MgaIniTimeUtcFields::TAccS field.
    ///     @li @b field_reserved2() for @ref MgaIniTimeUtcFields::Reserved2 field.
    ///     @li @b field_tAccNs() for @ref MgaIniTimeUtcFields::TAccNs field.
    COMMS_MSG_FIELDS_ACCESS(
        type,
        version,
        ref,
        leapSecs,
        year,
        month,
        day,
        hour,
        minute,
        second,
        reserved1,
        ns,
        tAccS,
        reserved2,
        tAccNs
    );
    
    // Compile time check for serialisation length.
    static const std::size_t MsgMinLen = Base::doMinLength();
    static const std::size_t MsgMaxLen = Base::doMaxLength();
    static_assert(MsgMinLen == 24U, "Unexpected min serialisation length");
    static_assert(MsgMaxLen == 24U, "Unexpected max serialisation length");
    
    /// @brief Name of the message.
    static const char* doName()
    {
        return "MGA-BDS-TIME_UTC";
    }
    
    
};

} // namespace message

} // namespace ublox


