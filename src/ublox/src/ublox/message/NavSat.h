/// @file
/// @brief Contains definition of <b>"NAV-SAT"</b> message and its fields.

#pragma once

#include <cstdint>
#include <tuple>
#include "comms/MessageBase.h"
#include "comms/field/ArrayList.h"
#include "comms/field/Bitfield.h"
#include "comms/field/BitmaskValue.h"
#include "comms/field/Bundle.h"
#include "comms/field/EnumValue.h"
#include "comms/field/IntValue.h"
#include "comms/options.h"
#include "ublox/DefaultOptions.h"
#include "ublox/MsgId.h"
#include "ublox/field/FieldBase.h"
#include "ublox/field/GnssId.h"
#include "ublox/field/Itow.h"
#include "ublox/field/Res2.h"

namespace ublox
{

namespace message
{

/// @brief Fields of @ref NavSat.
/// @tparam TOpt Extra options
/// @see @ref NavSat
/// @headerfile "ublox/message/NavSat.h"
template <typename TOpt = ublox::DefaultOptions>
struct NavSatFields
{
    /// @brief Definition of <b>"iTOW"</b> field.
    using Itow =
        ublox::field::Itow<
           TOpt
       >;
    
    /// @brief Definition of <b>"version"</b> field.
    struct Version : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t,
            comms::option::DefaultNumValue<1>,
            comms::option::ValidNumValue<1>
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "version";
        }
        
    };
    
    /// @brief Definition of <b>"numSvs"</b> field.
    struct NumSvs : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "numSvs";
        }
        
    };
    
    /// @brief Definition of <b>"reserved1"</b> field.
    struct Reserved1 : public
        ublox::field::Res2<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "reserved1";
        }
        
    };
    
    /// @brief Scope for all the member fields of @ref List list.
    struct ListMembers
    {
        /// @brief Scope for all the member fields of @ref Element bitfield.
        struct ElementMembers
        {
            /// @brief Definition of <b>"gnssId"</b> field.
            using GnssId =
                ublox::field::GnssId<
                   TOpt
               >;
            
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
            
            /// @brief Definition of <b>"cno"</b> field.
            struct Cno : public
                comms::field::IntValue<
                    ublox::field::FieldBase<>,
                    std::uint8_t
                >
            {
                /// @brief Name of the field.
                static const char* name()
                {
                    return "cno";
                }
                
            };
            
            /// @brief Definition of <b>"elev"</b> field.
            struct Elev : public
                comms::field::IntValue<
                    ublox::field::FieldBase<>,
                    std::int8_t,
                    comms::option::UnitsDegrees,
                    comms::option::ValidNumValueRange<-90, 90>
                >
            {
                /// @brief Name of the field.
                static const char* name()
                {
                    return "elev";
                }
                
            };
            
            /// @brief Definition of <b>"azim"</b> field.
            struct Azim : public
                comms::field::IntValue<
                    ublox::field::FieldBase<>,
                    std::int16_t,
                    comms::option::UnitsDegrees,
                    comms::option::ValidNumValueRange<0, 360>
                >
            {
                /// @brief Name of the field.
                static const char* name()
                {
                    return "azim";
                }
                
            };
            
            /// @brief Definition of <b>"prRes"</b> field.
            struct PrRes : public
                comms::field::IntValue<
                    ublox::field::FieldBase<>,
                    std::int16_t,
                    comms::option::ScalingRatio<1, 10>,
                    comms::option::UnitsMeters
                >
            {
                /// @brief Name of the field.
                static const char* name()
                {
                    return "prRes";
                }
                
            };
            
            /// @brief Scope for all the member fields of @ref Flags bitfield.
            struct FlagsMembers
            {
                /// @brief Values enumerator for @ref QualityInd field.
                enum class QualityIndVal : std::uint8_t
                {
                    NoSignal = 0, ///< value <b>no signal</b>.
                    Searching = 1, ///< value <b>searching</b>.
                    Acquired = 2, ///< value <b>signal acquired</b>.
                    Unusable = 3, ///< value <b>detected but unsable</b>.
                    CodeLocked = 4, ///< value <b>code locked</b>.
                    CodeCarrierLocked = 5, ///< value <b>code and carrier locked</b>.
                    CodeCarrierLocked2 = 6, ///< value <b>code and carrier locked</b>.
                    CodeCarrierLocked3 = 7, ///< value <b>code and carrier locked</b>.
                    
                };
                
                /// @brief Definition of <b>"qualityInd"</b> field.
                struct QualityInd : public
                    comms::field::EnumValue<
                        ublox::field::FieldBase<>,
                        QualityIndVal,
                        comms::option::FixedBitLength<3U>,
                        comms::option::ValidNumValueRange<0, 7>
                    >
                {
                    /// @brief Name of the field.
                    static const char* name()
                    {
                        return "qualityInd";
                    }
                    
                };
                
                /// @brief Definition of <b>""</b> field.
                class BitsLow : public
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
                    ///      @li @b BitIdx_svUsed, @b getBitValue_svUsed() and @b setBitValue_svUsed().
                    COMMS_BITMASK_BITS_SEQ(
                        svUsed
                    );
                    
                    /// @brief Name of the field.
                    static const char* name()
                    {
                        return "";
                    }
                    
                };
                
                /// @brief Values enumerator for @ref Health field.
                enum class HealthVal : std::uint8_t
                {
                    Unknown = 0, ///< value @b Unknown
                    Healthy = 1, ///< value @b Healthy
                    Unhealthy = 2, ///< value @b Unhealthy
                    
                };
                
                /// @brief Definition of <b>"health"</b> field.
                struct Health : public
                    comms::field::EnumValue<
                        ublox::field::FieldBase<>,
                        HealthVal,
                        comms::option::FixedBitLength<2U>,
                        comms::option::ValidNumValueRange<0, 2>
                    >
                {
                    /// @brief Name of the field.
                    static const char* name()
                    {
                        return "health";
                    }
                    
                };
                
                /// @brief Definition of <b>""</b> field.
                class BitsMid : public
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
                    ///      @li @b BitIdx_diffCorr, @b getBitValue_diffCorr() and @b setBitValue_diffCorr().
                    ///      @li @b BitIdx_smoothed, @b getBitValue_smoothed() and @b setBitValue_smoothed().
                    COMMS_BITMASK_BITS_SEQ(
                        diffCorr,
                        smoothed
                    );
                    
                    /// @brief Name of the field.
                    static const char* name()
                    {
                        return "";
                    }
                    
                };
                
                /// @brief Values enumerator for @ref OrbitSource field.
                enum class OrbitSourceVal : std::uint8_t
                {
                    NoOrbit = 0, ///< value <b>No orbit</b>.
                    Ephemeris = 1, ///< value @b Ephemeris
                    Almanac = 2, ///< value @b Almanac
                    AssistNowOffline = 3, ///< value <b>AssistNow Offline</b>.
                    AssistNowAutonomous = 4, ///< value <b>AssistNow Autonomous</b>.
                    
                };
                
                /// @brief Definition of <b>"orbitSource"</b> field.
                struct OrbitSource : public
                    comms::field::EnumValue<
                        ublox::field::FieldBase<>,
                        OrbitSourceVal,
                        comms::option::FixedBitLength<3U>,
                        comms::option::ValidNumValueRange<0, 4>
                    >
                {
                    /// @brief Name of the field.
                    static const char* name()
                    {
                        return "orbitSource";
                    }
                    
                };
                
                /// @brief Definition of <b>""</b> field.
                class BitsHigh : public
                    comms::field::BitmaskValue<
                        ublox::field::FieldBase<>,
                        comms::option::FixedBitLength<21U>,
                        comms::option::BitmaskReservedBits<0x1FF190UL, 0x0U>
                    >
                {
                    using Base = 
                        comms::field::BitmaskValue<
                            ublox::field::FieldBase<>,
                            comms::option::FixedBitLength<21U>,
                            comms::option::BitmaskReservedBits<0x1FF190UL, 0x0U>
                        >;
                public:
                    /// @brief Provide names for internal bits.
                    /// @details See definition of @b COMMS_BITMASK_BITS macro
                    ///     related to @b comms::field::BitmaskValue class from COMMS library
                    ///     for details.
                    ///
                    ///      The generated enum values:
                    ///      @li @b BitIdx_ephAvail.
                    ///      @li @b BitIdx_almAvail.
                    ///      @li @b BitIdx_anoAvail.
                    ///      @li @b BitIdx_aopAvail.
                    ///      @li @b BitIdx_sbasCorrUsed.
                    ///      @li @b BitIdx_rtcmCorrUsed.
                    ///      @li @b BitIdx_prCorrUsed.
                    ///      @li @b BitIdx_crCorrUsed.
                    ///      @li @b BitIdx_doCorrUsed.
                    COMMS_BITMASK_BITS(
                        ephAvail=0,
                        almAvail=1,
                        anoAvail=2,
                        aopAvail=3,
                        sbasCorrUsed=5,
                        rtcmCorrUsed=6,
                        prCorrUsed=9,
                        crCorrUsed=10,
                        doCorrUsed=11
                    );
                    
                    /// @brief Generates independent access functions for internal bits.
                    /// @details See definition of @b COMMS_BITMASK_BITS_ACCESS macro
                    ///     related to @b comms::field::BitmaskValue class from COMMS library
                    ///     for details.
                    ///
                    ///     The generated access functions are:
                    ///      @li @b getBitValue_ephAvail() and @b setBitValue_ephAvail().
                    ///      @li @b getBitValue_almAvail() and @b setBitValue_almAvail().
                    ///      @li @b getBitValue_anoAvail() and @b setBitValue_anoAvail().
                    ///      @li @b getBitValue_aopAvail() and @b setBitValue_aopAvail().
                    ///      @li @b getBitValue_sbasCorrUsed() and @b setBitValue_sbasCorrUsed().
                    ///      @li @b getBitValue_rtcmCorrUsed() and @b setBitValue_rtcmCorrUsed().
                    ///      @li @b getBitValue_prCorrUsed() and @b setBitValue_prCorrUsed().
                    ///      @li @b getBitValue_crCorrUsed() and @b setBitValue_crCorrUsed().
                    ///      @li @b getBitValue_doCorrUsed() and @b setBitValue_doCorrUsed().
                    COMMS_BITMASK_BITS_ACCESS(
                        ephAvail,
                        almAvail,
                        anoAvail,
                        aopAvail,
                        sbasCorrUsed,
                        rtcmCorrUsed,
                        prCorrUsed,
                        crCorrUsed,
                        doCorrUsed
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
                       QualityInd,
                       BitsLow,
                       Health,
                       BitsMid,
                       OrbitSource,
                       BitsHigh
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
                ///     @li @b field_qualityInd() - for FlagsMembers::QualityInd member field.
                ///     @li @b field_bitsLow() - for FlagsMembers::BitsLow member field.
                ///     @li @b field_health() - for FlagsMembers::Health member field.
                ///     @li @b field_bitsMid() - for FlagsMembers::BitsMid member field.
                ///     @li @b field_orbitSource() - for FlagsMembers::OrbitSource member field.
                ///     @li @b field_bitsHigh() - for FlagsMembers::BitsHigh member field.
                COMMS_FIELD_MEMBERS_ACCESS(
                    qualityInd,
                    bitsLow,
                    health,
                    bitsMid,
                    orbitSource,
                    bitsHigh
                );
                
                /// @brief Name of the field.
                static const char* name()
                {
                    return "flags";
                }
                
            };
            
            /// @brief All members bundled in @b std::tuple.
            using All =
                std::tuple<
                   GnssId,
                   Svid,
                   Cno,
                   Elev,
                   Azim,
                   PrRes,
                   Flags
                >;
        };
        
        /// @brief Definition of <b>""</b> field.
        class Element : public
            comms::field::Bundle<
                ublox::field::FieldBase<>,
                typename ElementMembers::All
            >
        {
            using Base = 
                comms::field::Bundle<
                    ublox::field::FieldBase<>,
                    typename ElementMembers::All
                >;
        public:
            /// @brief Allow access to internal fields.
            /// @details See definition of @b COMMS_FIELD_MEMBERS_ACCESS macro
            ///     related to @b comms::field::Bundle class from COMMS library
            ///     for details.
            ///
            ///     The generated access functions are:
            ///     @li @b field_gnssId() - for ElementMembers::GnssId member field.
            ///     @li @b field_svid() - for ElementMembers::Svid member field.
            ///     @li @b field_cno() - for ElementMembers::Cno member field.
            ///     @li @b field_elev() - for ElementMembers::Elev member field.
            ///     @li @b field_azim() - for ElementMembers::Azim member field.
            ///     @li @b field_prRes() - for ElementMembers::PrRes member field.
            ///     @li @b field_flags() - for ElementMembers::Flags member field.
            COMMS_FIELD_MEMBERS_ACCESS(
                gnssId,
                svid,
                cno,
                elev,
                azim,
                prRes,
                flags
            );
            
            /// @brief Name of the field.
            static const char* name()
            {
                return "";
            }
            
        };
        
    };
    
    /// @brief Definition of <b>"list"</b> field.
    struct List : public
        comms::field::ArrayList<
            ublox::field::FieldBase<>,
            typename ListMembers::Element,
            typename TOpt::message::NavSatFields::List,
            comms::option::SequenceSizeForcingEnabled
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "list";
        }
        
    };
    
    /// @brief All the fields bundled in std::tuple.
    using All = std::tuple<
        Itow,
        Version,
        NumSvs,
        Reserved1,
        List
    >;
};

/// @brief Definition of <b>"NAV-SAT"</b> message class.
/// @details
///     See @ref NavSatFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/NavSat.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class NavSat : public
    comms::MessageBase<
        TMsgBase,
        typename TOpt::message::NavSat,
        comms::option::StaticNumIdImpl<ublox::MsgId_NavSat>,
        comms::option::FieldsImpl<typename NavSatFields<TOpt>::All>,
        comms::option::MsgType<NavSat<TMsgBase, TOpt> >,
        comms::option::HasName,
        comms::option::HasCustomRefresh
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            typename TOpt::message::NavSat,
            comms::option::StaticNumIdImpl<ublox::MsgId_NavSat>,
            comms::option::FieldsImpl<typename NavSatFields<TOpt>::All>,
            comms::option::MsgType<NavSat<TMsgBase, TOpt> >,
            comms::option::HasName,
            comms::option::HasCustomRefresh
        >;

public:
    /// @brief Allow access to internal fields.
    /// @details See definition of @b COMMS_MSG_FIELDS_ACCESS macro
    ///     related to @b comms::MessageBase class from COMMS library
    ///     for details.
    ///
    ///     The generated functions are:
    ///     @li @b field_itow() for @ref NavSatFields::Itow field.
    ///     @li @b field_version() for @ref NavSatFields::Version field.
    ///     @li @b field_numSvs() for @ref NavSatFields::NumSvs field.
    ///     @li @b field_reserved1() for @ref NavSatFields::Reserved1 field.
    ///     @li @b field_list() for @ref NavSatFields::List field.
    COMMS_MSG_FIELDS_ACCESS(
        itow,
        version,
        numSvs,
        reserved1,
        list
    );
    
    // Compile time check for serialisation length.
    static const std::size_t MsgMinLen = Base::doMinLength();
    static_assert(MsgMinLen == 8U, "Unexpected min serialisation length");
    
    /// @brief Name of the message.
    static const char* doName()
    {
        return "NAV-SAT";
    }
    
    /// @brief Custom read functionality.
    template <typename TIter>
    comms::ErrorStatus doRead(TIter& iter, std::size_t len)
    {
        auto es = Base::template doReadUntilAndUpdateLen<FieldIdx_list>(iter, len);
        if (es != comms::ErrorStatus::Success) {
            return es;
        }
        
        field_list().forceReadElemCount(
            static_cast<std::size_t>(field_numSvs().value()));
        
        es = Base::template doReadFrom<FieldIdx_list>(iter, len);
        if (es != comms::ErrorStatus::Success) {
            return es;
        }
        
        return comms::ErrorStatus::Success;
    }
    
    /// @brief Custom refresh functionality.
    bool doRefresh()
    {
        bool updated = Base::doRefresh();
        updated = refresh_list() || updated;
        return updated;
    }
    
    
private:
    bool refresh_list()
    {
        bool updated = false;
        do {
            auto expectedValue = static_cast<std::size_t>(field_numSvs().value());
            auto realValue = field_list().value().size();
            if (expectedValue != realValue) {
                using PrefixValueType = typename std::decay<decltype(field_numSvs().value())>::type;
                field_numSvs().value() = static_cast<PrefixValueType>(realValue);
                updated = true;
            }
        } while (false);
        
        return updated;
        
    }
    
};

} // namespace message

} // namespace ublox


