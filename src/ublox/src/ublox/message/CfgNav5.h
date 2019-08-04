/// @file
/// @brief Contains definition of <b>"CFG-NAV5"</b> message and its fields.

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
#include "ublox/field/Res2.h"
#include "ublox/field/Res5.h"

namespace ublox
{

namespace message
{

/// @brief Fields of @ref CfgNav5.
/// @tparam TOpt Extra options
/// @see @ref CfgNav5
/// @headerfile "ublox/message/CfgNav5.h"
template <typename TOpt = ublox::DefaultOptions>
struct CfgNav5Fields
{
    /// @brief Definition of <b>"mask"</b> field.
    class Mask : public
        comms::field::BitmaskValue<
            ublox::field::FieldBase<>,
            comms::option::FixedLength<2U>,
            comms::option::BitmaskReservedBits<0xFA00U, 0x0U>
        >
    {
        using Base = 
            comms::field::BitmaskValue<
                ublox::field::FieldBase<>,
                comms::option::FixedLength<2U>,
                comms::option::BitmaskReservedBits<0xFA00U, 0x0U>
            >;
    public:
        /// @brief Provide names for internal bits.
        /// @details See definition of @b COMMS_BITMASK_BITS macro
        ///     related to @b comms::field::BitmaskValue class from COMMS library
        ///     for details.
        ///
        ///      The generated enum values:
        ///      @li @b BitIdx_dyn.
        ///      @li @b BitIdx_minEl.
        ///      @li @b BitIdx_posFixMode.
        ///      @li @b BitIdx_drLim.
        ///      @li @b BitIdx_posMask.
        ///      @li @b BitIdx_timeMask.
        ///      @li @b BitIdx_staticHoldMask.
        ///      @li @b BitIdx_dgpsMask.
        ///      @li @b BitIdx_cnoThreshold.
        ///      @li @b BitIdx_utc.
        COMMS_BITMASK_BITS(
            dyn=0,
            minEl=1,
            posFixMode=2,
            drLim=3,
            posMask=4,
            timeMask=5,
            staticHoldMask=6,
            dgpsMask=7,
            cnoThreshold=8,
            utc=10
        );
        
        /// @brief Generates independent access functions for internal bits.
        /// @details See definition of @b COMMS_BITMASK_BITS_ACCESS macro
        ///     related to @b comms::field::BitmaskValue class from COMMS library
        ///     for details.
        ///
        ///     The generated access functions are:
        ///      @li @b getBitValue_dyn() and @b setBitValue_dyn().
        ///      @li @b getBitValue_minEl() and @b setBitValue_minEl().
        ///      @li @b getBitValue_posFixMode() and @b setBitValue_posFixMode().
        ///      @li @b getBitValue_drLim() and @b setBitValue_drLim().
        ///      @li @b getBitValue_posMask() and @b setBitValue_posMask().
        ///      @li @b getBitValue_timeMask() and @b setBitValue_timeMask().
        ///      @li @b getBitValue_staticHoldMask() and @b setBitValue_staticHoldMask().
        ///      @li @b getBitValue_dgpsMask() and @b setBitValue_dgpsMask().
        ///      @li @b getBitValue_cnoThreshold() and @b setBitValue_cnoThreshold().
        ///      @li @b getBitValue_utc() and @b setBitValue_utc().
        COMMS_BITMASK_BITS_ACCESS(
            dyn,
            minEl,
            posFixMode,
            drLim,
            posMask,
            timeMask,
            staticHoldMask,
            dgpsMask,
            cnoThreshold,
            utc
        );
        
        /// @brief Name of the field.
        static const char* name()
        {
            return "mask";
        }
        
    };
    
    /// @brief Values enumerator for @ref DynModel field.
    enum class DynModelVal : std::uint8_t
    {
        Portable = 0, ///< value @b Portable
        Stationary = 2, ///< value @b Stationary
        Pedestrian = 3, ///< value @b Pedestrian
        Automotive = 4, ///< value @b Automotive
        Sea = 5, ///< value @b Sea
        Airborne1G = 6, ///< value <b>Airborne <1g accel</b>.
        Airborne2G = 7, ///< value <b>Airborne <2g accel</b>.
        Airborne4G = 8, ///< value <b>Airborne <4g accel</b>.
        WristWatch = 9, ///< value <b>Wrist watch</b>.
        
    };
    
    /// @brief Definition of <b>"dynModel"</b> field.
    struct DynModel : public
        comms::field::EnumValue<
            ublox::field::FieldBase<>,
            DynModelVal,
            comms::option::ValidNumValue<0>,
            comms::option::ValidNumValueRange<2, 9>
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "dynModel";
        }
        
    };
    
    /// @brief Values enumerator for @ref FixMode field.
    enum class FixModeVal : std::uint8_t
    {
        Only2D = 1, ///< value <b>2D Only</b>.
        Only3D = 2, ///< value <b>2D Only</b>.
        Auto = 3, ///< value <b>Auto 2D/3D</b>.
        
    };
    
    /// @brief Definition of <b>"fixMode"</b> field.
    struct FixMode : public
        comms::field::EnumValue<
            ublox::field::FieldBase<>,
            FixModeVal,
            comms::option::ValidNumValueRange<1, 3>
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "fixMode";
        }
        
    };
    
    /// @brief Definition of <b>"fixedAlt"</b> field.
    struct FixedAlt : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::int32_t,
            comms::option::ScalingRatio<1, 100>,
            comms::option::UnitsMeters
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "fixedAlt";
        }
        
    };
    
    /// @brief Definition of <b>"fixedAltVar"</b> field.
    struct FixedAltVar : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint32_t,
            comms::option::ScalingRatio<1, 10000>
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "fixedAltVar";
        }
        
    };
    
    /// @brief Definition of <b>"minElev"</b> field.
    struct MinElev : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::int8_t,
            comms::option::UnitsDegrees
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "minElev";
        }
        
    };
    
    /// @brief Definition of <b>"drLimit"</b> field.
    struct DrLimit : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t,
            comms::option::UnitsSeconds
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "drLimit";
        }
        
    };
    
    /// @brief Definition of <b>"pDop"</b> field.
    struct PDop : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint16_t,
            comms::option::ScalingRatio<1, 10>
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "pDop";
        }
        
    };
    
    /// @brief Definition of <b>"tDop"</b> field.
    struct TDop : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint16_t,
            comms::option::ScalingRatio<1, 10>
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "tDop";
        }
        
    };
    
    /// @brief Definition of <b>"pAcc"</b> field.
    struct PAcc : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint16_t,
            comms::option::UnitsMeters
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "pAcc";
        }
        
    };
    
    /// @brief Definition of <b>"tAcc"</b> field.
    struct TAcc : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint16_t,
            comms::option::UnitsMeters
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "tAcc";
        }
        
    };
    
    /// @brief Definition of <b>"staticHoldThresh"</b> field.
    struct StaticHoldThresh : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t,
            comms::option::UnitsCentimetersPerSecond
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "staticHoldThresh";
        }
        
    };
    
    /// @brief Definition of <b>"dgnssTimeout"</b> field.
    struct DgnssTimeout : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t,
            comms::option::UnitsSeconds
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "dgnssTimeout";
        }
        
    };
    
    /// @brief Definition of <b>"cnoThreshNumSVs"</b> field.
    struct CnoThreshNumSVs : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "cnoThreshNumSVs";
        }
        
    };
    
    /// @brief Definition of <b>"cnoThresh"</b> field.
    struct CnoThresh : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "cnoThresh";
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
    
    /// @brief Definition of <b>"staticHoldMaxDist"</b> field.
    struct StaticHoldMaxDist : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint16_t,
            comms::option::UnitsMeters
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "staticHoldMaxDist";
        }
        
    };
    
    /// @brief Values enumerator for @ref UtcStandard field.
    enum class UtcStandardVal : std::uint8_t
    {
        Automatic = 0, ///< value @b Automatic
        GPS = 3, ///< value @b GPS
        GLONASS = 6, ///< value @b GLONASS
        BeiDou = 7, ///< value @b BeiDou
        
    };
    
    /// @brief Definition of <b>"utcStandard"</b> field.
    struct UtcStandard : public
        comms::field::EnumValue<
            ublox::field::FieldBase<>,
            UtcStandardVal,
            comms::option::ValidNumValue<0>,
            comms::option::ValidNumValue<3>,
            comms::option::ValidNumValueRange<6, 7>
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "utcStandard";
        }
        
    };
    
    /// @brief Definition of <b>"reserved2"</b> field.
    struct Reserved2 : public
        ublox::field::Res5<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "reserved2";
        }
        
    };
    
    /// @brief All the fields bundled in std::tuple.
    using All = std::tuple<
        Mask,
        DynModel,
        FixMode,
        FixedAlt,
        FixedAltVar,
        MinElev,
        DrLimit,
        PDop,
        TDop,
        PAcc,
        TAcc,
        StaticHoldThresh,
        DgnssTimeout,
        CnoThreshNumSVs,
        CnoThresh,
        Reserved1,
        StaticHoldMaxDist,
        UtcStandard,
        Reserved2
    >;
};

/// @brief Definition of <b>"CFG-NAV5"</b> message class.
/// @details
///     See @ref CfgNav5Fields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/CfgNav5.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class CfgNav5 : public
    comms::MessageBase<
        TMsgBase,
        comms::option::StaticNumIdImpl<ublox::MsgId_CfgNav5>,
        comms::option::FieldsImpl<typename CfgNav5Fields<TOpt>::All>,
        comms::option::MsgType<CfgNav5<TMsgBase, TOpt> >,
        comms::option::HasName
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            comms::option::StaticNumIdImpl<ublox::MsgId_CfgNav5>,
            comms::option::FieldsImpl<typename CfgNav5Fields<TOpt>::All>,
            comms::option::MsgType<CfgNav5<TMsgBase, TOpt> >,
            comms::option::HasName
        >;

public:
    /// @brief Allow access to internal fields.
    /// @details See definition of @b COMMS_MSG_FIELDS_ACCESS macro
    ///     related to @b comms::MessageBase class from COMMS library
    ///     for details.
    ///
    ///     The generated functions are:
    ///     @li @b field_mask() for @ref CfgNav5Fields::Mask field.
    ///     @li @b field_dynModel() for @ref CfgNav5Fields::DynModel field.
    ///     @li @b field_fixMode() for @ref CfgNav5Fields::FixMode field.
    ///     @li @b field_fixedAlt() for @ref CfgNav5Fields::FixedAlt field.
    ///     @li @b field_fixedAltVar() for @ref CfgNav5Fields::FixedAltVar field.
    ///     @li @b field_minElev() for @ref CfgNav5Fields::MinElev field.
    ///     @li @b field_drLimit() for @ref CfgNav5Fields::DrLimit field.
    ///     @li @b field_pDop() for @ref CfgNav5Fields::PDop field.
    ///     @li @b field_tDop() for @ref CfgNav5Fields::TDop field.
    ///     @li @b field_pAcc() for @ref CfgNav5Fields::PAcc field.
    ///     @li @b field_tAcc() for @ref CfgNav5Fields::TAcc field.
    ///     @li @b field_staticHoldThresh() for @ref CfgNav5Fields::StaticHoldThresh field.
    ///     @li @b field_dgnssTimeout() for @ref CfgNav5Fields::DgnssTimeout field.
    ///     @li @b field_cnoThreshNumSVs() for @ref CfgNav5Fields::CnoThreshNumSVs field.
    ///     @li @b field_cnoThresh() for @ref CfgNav5Fields::CnoThresh field.
    ///     @li @b field_reserved1() for @ref CfgNav5Fields::Reserved1 field.
    ///     @li @b field_staticHoldMaxDist() for @ref CfgNav5Fields::StaticHoldMaxDist field.
    ///     @li @b field_utcStandard() for @ref CfgNav5Fields::UtcStandard field.
    ///     @li @b field_reserved2() for @ref CfgNav5Fields::Reserved2 field.
    COMMS_MSG_FIELDS_ACCESS(
        mask,
        dynModel,
        fixMode,
        fixedAlt,
        fixedAltVar,
        minElev,
        drLimit,
        pDop,
        tDop,
        pAcc,
        tAcc,
        staticHoldThresh,
        dgnssTimeout,
        cnoThreshNumSVs,
        cnoThresh,
        reserved1,
        staticHoldMaxDist,
        utcStandard,
        reserved2
    );
    
    // Compile time check for serialisation length.
    static const std::size_t MsgMinLen = Base::doMinLength();
    static const std::size_t MsgMaxLen = Base::doMaxLength();
    static_assert(MsgMinLen == 36U, "Unexpected min serialisation length");
    static_assert(MsgMaxLen == 36U, "Unexpected max serialisation length");
    
    /// @brief Name of the message.
    static const char* doName()
    {
        return "CFG-NAV5";
    }
    
    
};

} // namespace message

} // namespace ublox


