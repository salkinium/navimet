/// @file
/// @brief Contains definition of <b>"NAV-PVT"</b> message and its fields.

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
#include "ublox/field/Dop.h"
#include "ublox/field/FieldBase.h"
#include "ublox/field/GpsFix.h"
#include "ublox/field/HAcc.h"
#include "ublox/field/HMSL.h"
#include "ublox/field/Heading.h"
#include "ublox/field/HeadingAcc.h"
#include "ublox/field/Height.h"
#include "ublox/field/Hour.h"
#include "ublox/field/Itow.h"
#include "ublox/field/Lat.h"
#include "ublox/field/Lon.h"
#include "ublox/field/Min.h"
#include "ublox/field/Month.h"
#include "ublox/field/Res2.h"
#include "ublox/field/Res4.h"
#include "ublox/field/Sec.h"
#include "ublox/field/VAcc.h"
#include "ublox/field/Vel.h"
#include "ublox/field/Year.h"

namespace ublox
{

namespace message
{

/// @brief Fields of @ref NavPvt.
/// @tparam TOpt Extra options
/// @see @ref NavPvt
/// @headerfile "ublox/message/NavPvt.h"
template <typename TOpt = ublox::DefaultOptions>
struct NavPvtFields
{
    /// @brief Definition of <b>"iTOW"</b> field.
    using Itow =
        ublox::field::Itow<
           TOpt
       >;
    
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
    
    /// @brief Definition of <b>"min"</b> field.
    using Min =
        ublox::field::Min<
           TOpt
       >;
    
    /// @brief Definition of <b>"sec"</b> field.
    using Sec =
        ublox::field::Sec<
           TOpt
       >;
    
    /// @brief Definition of <b>"valid"</b> field.
    class Valid : public
        comms::field::BitmaskValue<
            ublox::field::FieldBase<>,
            comms::option::FixedLength<1U>,
            comms::option::BitmaskReservedBits<0xF8U, 0x0U>
        >
    {
        using Base = 
            comms::field::BitmaskValue<
                ublox::field::FieldBase<>,
                comms::option::FixedLength<1U>,
                comms::option::BitmaskReservedBits<0xF8U, 0x0U>
            >;
    public:
        /// @brief Provides names and generates access functions for internal bits.
        /// @details See definition of @b COMMS_BITMASK_BITS_SEQ macro
        ///     related to @b comms::field::BitmaskValue class from COMMS library
        ///     for details.
        ///
        ///      The generated enum values and functions are:
        ///      @li @b BitIdx_validDate, @b getBitValue_validDate() and @b setBitValue_validDate().
        ///      @li @b BitIdx_validTime, @b getBitValue_validTime() and @b setBitValue_validTime().
        ///      @li @b BitIdx_fullyResolved, @b getBitValue_fullyResolved() and @b setBitValue_fullyResolved().
        COMMS_BITMASK_BITS_SEQ(
            validDate,
            validTime,
            fullyResolved
        );
        
        /// @brief Name of the field.
        static const char* name()
        {
            return "valid";
        }
        
    };
    
    /// @brief Definition of <b>"tAcc"</b> field.
    struct TAcc : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint32_t,
            comms::option::UnitsNanoseconds
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "tAcc";
        }
        
    };
    
    /// @brief Definition of <b>"nano"</b> field.
    struct Nano : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::int32_t,
            comms::option::UnitsNanoseconds
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "nano";
        }
        
    };
    
    /// @brief Definition of <b>"fixType"</b> field.
    struct FixType : public
        ublox::field::GpsFix<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "fixType";
        }
        
    };
    
    /// @brief Scope for all the member fields of @ref Flags bitfield.
    struct FlagsMembers
    {
        /// @brief Definition of <b>""</b> field.
        class LowBits : public
            comms::field::BitmaskValue<
                ublox::field::FieldBase<>,
                comms::option::FixedBitLength<2U>
            >
        {
            using Base = 
                comms::field::BitmaskValue<
                    ublox::field::FieldBase<>,
                    comms::option::FixedBitLength<2U>
                >;
        public:
            /// @brief Provides names and generates access functions for internal bits.
            /// @details See definition of @b COMMS_BITMASK_BITS_SEQ macro
            ///     related to @b comms::field::BitmaskValue class from COMMS library
            ///     for details.
            ///
            ///      The generated enum values and functions are:
            ///      @li @b BitIdx_gnssFixOK, @b getBitValue_gnssFixOK() and @b setBitValue_gnssFixOK().
            ///      @li @b BitIdx_diffSoln, @b getBitValue_diffSoln() and @b setBitValue_diffSoln().
            COMMS_BITMASK_BITS_SEQ(
                gnssFixOK,
                diffSoln
            );
            
            /// @brief Name of the field.
            static const char* name()
            {
                return "";
            }
            
        };
        
        /// @brief Values enumerator for @ref PsmState field.
        enum class PsmStateVal : std::uint8_t
        {
            NotAvailable = 0, ///< value <b>N/A</b>.
            Enabled = 1, ///< value <b>ENABLED</b>.
            Acquisition = 2, ///< value <b>ACQUISITION</b>.
            Tracking = 3, ///< value <b>TRACKING</b>.
            PowerOptTracking = 4, ///< value <b>POWER OPTIMIZED TRACKING</b>.
            Inactive = 5, ///< value <b>INACTIVE</b>.
            
        };
        
        /// @brief Definition of <b>"psmState"</b> field.
        struct PsmState : public
            comms::field::EnumValue<
                ublox::field::FieldBase<>,
                PsmStateVal,
                comms::option::FixedBitLength<3U>,
                comms::option::ValidNumValueRange<0, 5>
            >
        {
            /// @brief Name of the field.
            static const char* name()
            {
                return "psmState";
            }
            
        };
        
        /// @brief Definition of <b>""</b> field.
        class MidBits : public
            comms::field::BitmaskValue<
                ublox::field::FieldBase<>,
                comms::option::FixedBitLength<1U>
            >
        {
            using Base = 
                comms::field::BitmaskValue<
                    ublox::field::FieldBase<>,
                    comms::option::FixedBitLength<1U>
                >;
        public:
            /// @brief Provides names and generates access functions for internal bits.
            /// @details See definition of @b COMMS_BITMASK_BITS_SEQ macro
            ///     related to @b comms::field::BitmaskValue class from COMMS library
            ///     for details.
            ///
            ///      The generated enum values and functions are:
            ///      @li @b BitIdx_headVehValid, @b getBitValue_headVehValid() and @b setBitValue_headVehValid().
            COMMS_BITMASK_BITS_SEQ(
                headVehValid
            );
            
            /// @brief Name of the field.
            static const char* name()
            {
                return "";
            }
            
        };
        
        /// @brief Values enumerator for @ref CarrSoln field.
        enum class CarrSolnVal : std::uint8_t
        {
            NoCarrier = 0, ///< value <b>No carrier</b>.
            Float = 1, ///< value <b>Float solution</b>.
            Fixed = 2, ///< value <b>Fixed solution</b>.
            
        };
        
        /// @brief Definition of <b>"carrSoln"</b> field.
        struct CarrSoln : public
            comms::field::EnumValue<
                ublox::field::FieldBase<>,
                CarrSolnVal,
                comms::option::FixedBitLength<2U>,
                comms::option::ValidNumValueRange<0, 2>
            >
        {
            /// @brief Name of the field.
            static const char* name()
            {
                return "carrSoln";
            }
            
        };
        
        /// @brief All members bundled in @b std::tuple.
        using All =
            std::tuple<
               LowBits,
               PsmState,
               MidBits,
               CarrSoln
            >;
    };
    
    /// @brief Definition of <b>"flags"</b> field.
    class Flags : public
        comms::field::Bitfield<
            ublox::field::FieldBase<>,
            typename FlagsMembers::All
        >
    {
        using Base = 
            comms::field::Bitfield<
                ublox::field::FieldBase<>,
                typename FlagsMembers::All
            >;
    public:
        /// @brief Allow access to internal fields.
        /// @details See definition of @b COMMS_FIELD_MEMBERS_ACCESS macro
        ///     related to @b comms::field::Bitfield class from COMMS library
        ///     for details.
        ///
        ///      The generated access functions are:
        ///     @li @b field_lowBits() - for FlagsMembers::LowBits member field.
        ///     @li @b field_psmState() - for FlagsMembers::PsmState member field.
        ///     @li @b field_midBits() - for FlagsMembers::MidBits member field.
        ///     @li @b field_carrSoln() - for FlagsMembers::CarrSoln member field.
        COMMS_FIELD_MEMBERS_ACCESS(
            lowBits,
            psmState,
            midBits,
            carrSoln
        );
        
        /// @brief Name of the field.
        static const char* name()
        {
            return "flags";
        }
        
    };
    
    /// @brief Definition of <b>"flags2"</b> field.
    class Flags2 : public
        comms::field::BitmaskValue<
            ublox::field::FieldBase<>,
            comms::option::FixedLength<1U>,
            comms::option::BitmaskReservedBits<0x1FU, 0x0U>
        >
    {
        using Base = 
            comms::field::BitmaskValue<
                ublox::field::FieldBase<>,
                comms::option::FixedLength<1U>,
                comms::option::BitmaskReservedBits<0x1FU, 0x0U>
            >;
    public:
        /// @brief Provide names for internal bits.
        /// @details See definition of @b COMMS_BITMASK_BITS macro
        ///     related to @b comms::field::BitmaskValue class from COMMS library
        ///     for details.
        ///
        ///      The generated enum values:
        ///      @li @b BitIdx_confirmedAvai.
        ///      @li @b BitIdx_confirmedDate.
        ///      @li @b BitIdx_confirmedTime.
        COMMS_BITMASK_BITS(
            confirmedAvai=5,
            confirmedDate=6,
            confirmedTime=7
        );
        
        /// @brief Generates independent access functions for internal bits.
        /// @details See definition of @b COMMS_BITMASK_BITS_ACCESS macro
        ///     related to @b comms::field::BitmaskValue class from COMMS library
        ///     for details.
        ///
        ///     The generated access functions are:
        ///      @li @b getBitValue_confirmedAvai() and @b setBitValue_confirmedAvai().
        ///      @li @b getBitValue_confirmedDate() and @b setBitValue_confirmedDate().
        ///      @li @b getBitValue_confirmedTime() and @b setBitValue_confirmedTime().
        COMMS_BITMASK_BITS_ACCESS(
            confirmedAvai,
            confirmedDate,
            confirmedTime
        );
        
        /// @brief Name of the field.
        static const char* name()
        {
            return "flags2";
        }
        
    };
    
    /// @brief Definition of <b>"numSV"</b> field.
    struct NumSV : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "numSV";
        }
        
    };
    
    /// @brief Definition of <b>"lon"</b> field.
    using Lon =
        ublox::field::Lon<
           TOpt
       >;
    
    /// @brief Definition of <b>"lat"</b> field.
    using Lat =
        ublox::field::Lat<
           TOpt
       >;
    
    /// @brief Definition of <b>"height"</b> field.
    using Height =
        ublox::field::Height<
           TOpt
       >;
    
    /// @brief Definition of <b>"hMSL"</b> field.
    using HMSL =
        ublox::field::HMSL<
           TOpt
       >;
    
    /// @brief Definition of <b>"hAcc"</b> field.
    using HAcc =
        ublox::field::HAcc<
           TOpt
       >;
    
    /// @brief Definition of <b>"vAcc"</b> field.
    using VAcc =
        ublox::field::VAcc<
           TOpt
       >;
    
    /// @brief Definition of <b>"velN"</b> field.
    struct VelN : public
        ublox::field::Vel<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "velN";
        }
        
    };
    
    /// @brief Definition of <b>"velE"</b> field.
    struct VelE : public
        ublox::field::Vel<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "velE";
        }
        
    };
    
    /// @brief Definition of <b>"velD"</b> field.
    struct VelD : public
        ublox::field::Vel<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "velD";
        }
        
    };
    
    /// @brief Definition of <b>"gSpeed"</b> field.
    struct GSpeed : public
        ublox::field::Vel<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "gSpeed";
        }
        
    };
    
    /// @brief Definition of <b>"heading"</b> field.
    using Heading =
        ublox::field::Heading<
           TOpt
       >;
    
    /// @brief Definition of <b>"sAcc"</b> field.
    struct SAcc : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint32_t,
            comms::option::UnitsMillimetersPerSecond
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "sAcc";
        }
        
    };
    
    /// @brief Definition of <b>"headingAcc"</b> field.
    using HeadingAcc =
        ublox::field::HeadingAcc<
           TOpt
       >;
    
    /// @brief Definition of <b>"pDop"</b> field.
    struct PDop : public
        ublox::field::Dop<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "pDop";
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
        Year,
        Month,
        Day,
        Hour,
        Min,
        Sec,
        Valid,
        TAcc,
        Nano,
        FixType,
        Flags,
        Flags2,
        NumSV,
        Lon,
        Lat,
        Height,
        HMSL,
        HAcc,
        VAcc,
        VelN,
        VelE,
        VelD,
        GSpeed,
        Heading,
        SAcc,
        HeadingAcc,
        PDop,
        Reserved2,
        Reserved3
    >;
};

/// @brief Definition of <b>"NAV-PVT"</b> message class.
/// @details
///     See @ref NavPvtFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/NavPvt.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class NavPvt : public
    comms::MessageBase<
        TMsgBase,
        typename TOpt::message::NavPvt,
        comms::option::StaticNumIdImpl<ublox::MsgId_NavPvt>,
        comms::option::FieldsImpl<typename NavPvtFields<TOpt>::All>,
        comms::option::MsgType<NavPvt<TMsgBase, TOpt> >,
        comms::option::HasName
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            typename TOpt::message::NavPvt,
            comms::option::StaticNumIdImpl<ublox::MsgId_NavPvt>,
            comms::option::FieldsImpl<typename NavPvtFields<TOpt>::All>,
            comms::option::MsgType<NavPvt<TMsgBase, TOpt> >,
            comms::option::HasName
        >;

public:
    /// @brief Allow access to internal fields.
    /// @details See definition of @b COMMS_MSG_FIELDS_ACCESS macro
    ///     related to @b comms::MessageBase class from COMMS library
    ///     for details.
    ///
    ///     The generated functions are:
    ///     @li @b field_itow() for @ref NavPvtFields::Itow field.
    ///     @li @b field_year() for @ref NavPvtFields::Year field.
    ///     @li @b field_month() for @ref NavPvtFields::Month field.
    ///     @li @b field_day() for @ref NavPvtFields::Day field.
    ///     @li @b field_hour() for @ref NavPvtFields::Hour field.
    ///     @li @b field_min() for @ref NavPvtFields::Min field.
    ///     @li @b field_sec() for @ref NavPvtFields::Sec field.
    ///     @li @b field_valid() for @ref NavPvtFields::Valid field.
    ///     @li @b field_tAcc() for @ref NavPvtFields::TAcc field.
    ///     @li @b field_nano() for @ref NavPvtFields::Nano field.
    ///     @li @b field_fixType() for @ref NavPvtFields::FixType field.
    ///     @li @b field_flags() for @ref NavPvtFields::Flags field.
    ///     @li @b field_flags2() for @ref NavPvtFields::Flags2 field.
    ///     @li @b field_numSV() for @ref NavPvtFields::NumSV field.
    ///     @li @b field_lon() for @ref NavPvtFields::Lon field.
    ///     @li @b field_lat() for @ref NavPvtFields::Lat field.
    ///     @li @b field_height() for @ref NavPvtFields::Height field.
    ///     @li @b field_hMSL() for @ref NavPvtFields::HMSL field.
    ///     @li @b field_hAcc() for @ref NavPvtFields::HAcc field.
    ///     @li @b field_vAcc() for @ref NavPvtFields::VAcc field.
    ///     @li @b field_velN() for @ref NavPvtFields::VelN field.
    ///     @li @b field_velE() for @ref NavPvtFields::VelE field.
    ///     @li @b field_velD() for @ref NavPvtFields::VelD field.
    ///     @li @b field_gSpeed() for @ref NavPvtFields::GSpeed field.
    ///     @li @b field_heading() for @ref NavPvtFields::Heading field.
    ///     @li @b field_sAcc() for @ref NavPvtFields::SAcc field.
    ///     @li @b field_headingAcc() for @ref NavPvtFields::HeadingAcc field.
    ///     @li @b field_pDop() for @ref NavPvtFields::PDop field.
    ///     @li @b field_reserved2() for @ref NavPvtFields::Reserved2 field.
    ///     @li @b field_reserved3() for @ref NavPvtFields::Reserved3 field.
    COMMS_MSG_FIELDS_ACCESS(
        itow,
        year,
        month,
        day,
        hour,
        min,
        sec,
        valid,
        tAcc,
        nano,
        fixType,
        flags,
        flags2,
        numSV,
        lon,
        lat,
        height,
        hMSL,
        hAcc,
        vAcc,
        velN,
        velE,
        velD,
        gSpeed,
        heading,
        sAcc,
        headingAcc,
        pDop,
        reserved2,
        reserved3
    );
    
    // Compile time check for serialisation length.
    static const std::size_t MsgMinLen = Base::doMinLength();
    static const std::size_t MsgMaxLen = Base::doMaxLength();
    static_assert(MsgMinLen == 84U, "Unexpected min serialisation length");
    static_assert(MsgMaxLen == 84U, "Unexpected max serialisation length");
    
    /// @brief Name of the message.
    static const char* doName()
    {
        return "NAV-PVT";
    }
    
    
};

} // namespace message

} // namespace ublox


