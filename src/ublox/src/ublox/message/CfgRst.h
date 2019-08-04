/// @file
/// @brief Contains definition of <b>"CFG-RST"</b> message and its fields.

#pragma once

#include <cstdint>
#include <tuple>
#include "comms/MessageBase.h"
#include "comms/field/BitmaskValue.h"
#include "comms/field/EnumValue.h"
#include "comms/options.h"
#include "ublox/DefaultOptions.h"
#include "ublox/MsgId.h"
#include "ublox/field/FieldBase.h"
#include "ublox/field/Res1.h"

namespace ublox
{

namespace message
{

/// @brief Fields of @ref CfgRst.
/// @tparam TOpt Extra options
/// @see @ref CfgRst
/// @headerfile "ublox/message/CfgRst.h"
template <typename TOpt = ublox::DefaultOptions>
struct CfgRstFields
{
    /// @brief Definition of <b>"navBbrMask"</b> field.
    class NavBbrMask : public
        comms::field::BitmaskValue<
            ublox::field::FieldBase<>,
            comms::option::FixedLength<2U>,
            comms::option::BitmaskReservedBits<0x7E00U, 0x0U>
        >
    {
        using Base = 
            comms::field::BitmaskValue<
                ublox::field::FieldBase<>,
                comms::option::FixedLength<2U>,
                comms::option::BitmaskReservedBits<0x7E00U, 0x0U>
            >;
    public:
        /// @brief Provide names for internal bits.
        /// @details See definition of @b COMMS_BITMASK_BITS macro
        ///     related to @b comms::field::BitmaskValue class from COMMS library
        ///     for details.
        ///
        ///      The generated enum values:
        ///      @li @b BitIdx_eph.
        ///      @li @b BitIdx_alm.
        ///      @li @b BitIdx_health.
        ///      @li @b BitIdx_klob.
        ///      @li @b BitIdx_pos.
        ///      @li @b BitIdx_clkd.
        ///      @li @b BitIdx_osc.
        ///      @li @b BitIdx_utc.
        ///      @li @b BitIdx_rtc.
        ///      @li @b BitIdx_aop.
        COMMS_BITMASK_BITS(
            eph=0,
            alm=1,
            health=2,
            klob=3,
            pos=4,
            clkd=5,
            osc=6,
            utc=7,
            rtc=8,
            aop=15
        );
        
        /// @brief Generates independent access functions for internal bits.
        /// @details See definition of @b COMMS_BITMASK_BITS_ACCESS macro
        ///     related to @b comms::field::BitmaskValue class from COMMS library
        ///     for details.
        ///
        ///     The generated access functions are:
        ///      @li @b getBitValue_eph() and @b setBitValue_eph().
        ///      @li @b getBitValue_alm() and @b setBitValue_alm().
        ///      @li @b getBitValue_health() and @b setBitValue_health().
        ///      @li @b getBitValue_klob() and @b setBitValue_klob().
        ///      @li @b getBitValue_pos() and @b setBitValue_pos().
        ///      @li @b getBitValue_clkd() and @b setBitValue_clkd().
        ///      @li @b getBitValue_osc() and @b setBitValue_osc().
        ///      @li @b getBitValue_utc() and @b setBitValue_utc().
        ///      @li @b getBitValue_rtc() and @b setBitValue_rtc().
        ///      @li @b getBitValue_aop() and @b setBitValue_aop().
        COMMS_BITMASK_BITS_ACCESS(
            eph,
            alm,
            health,
            klob,
            pos,
            clkd,
            osc,
            utc,
            rtc,
            aop
        );
        
        /// @brief Name of the field.
        static const char* name()
        {
            return "navBbrMask";
        }
        
    };
    
    /// @brief Values enumerator for @ref ResetMode field.
    enum class ResetModeVal : std::uint8_t
    {
        Hardware = 0, ///< value @b Hardware
        Software = 1, ///< value @b Software
        SoftwareGnssOnly = 2, ///< value <b>Software (GNSS only)</b>.
        HardwareAfterShutdown = 4, ///< value <b>Hardware (after shutdown)</b>.
        GnssStop = 8, ///< value <b>GNSS stop</b>.
        GnssStart = 9, ///< value <b>GNSS start</b>.
        
    };
    
    /// @brief Definition of <b>"resetMode"</b> field.
    struct ResetMode : public
        comms::field::EnumValue<
            ublox::field::FieldBase<>,
            ResetModeVal,
            comms::option::ValidNumValueRange<0, 2>,
            comms::option::ValidNumValue<4>,
            comms::option::ValidNumValueRange<8, 9>
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "resetMode";
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
    
    /// @brief All the fields bundled in std::tuple.
    using All = std::tuple<
        NavBbrMask,
        ResetMode,
        Reserved1
    >;
};

/// @brief Definition of <b>"CFG-RST"</b> message class.
/// @details
///     See @ref CfgRstFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/CfgRst.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class CfgRst : public
    comms::MessageBase<
        TMsgBase,
        typename TOpt::message::CfgRst,
        comms::option::StaticNumIdImpl<ublox::MsgId_CfgRst>,
        comms::option::FieldsImpl<typename CfgRstFields<TOpt>::All>,
        comms::option::MsgType<CfgRst<TMsgBase, TOpt> >,
        comms::option::HasName
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            typename TOpt::message::CfgRst,
            comms::option::StaticNumIdImpl<ublox::MsgId_CfgRst>,
            comms::option::FieldsImpl<typename CfgRstFields<TOpt>::All>,
            comms::option::MsgType<CfgRst<TMsgBase, TOpt> >,
            comms::option::HasName
        >;

public:
    /// @brief Allow access to internal fields.
    /// @details See definition of @b COMMS_MSG_FIELDS_ACCESS macro
    ///     related to @b comms::MessageBase class from COMMS library
    ///     for details.
    ///
    ///     The generated functions are:
    ///     @li @b field_navBbrMask() for @ref CfgRstFields::NavBbrMask field.
    ///     @li @b field_resetMode() for @ref CfgRstFields::ResetMode field.
    ///     @li @b field_reserved1() for @ref CfgRstFields::Reserved1 field.
    COMMS_MSG_FIELDS_ACCESS(
        navBbrMask,
        resetMode,
        reserved1
    );
    
    // Compile time check for serialisation length.
    static const std::size_t MsgMinLen = Base::doMinLength();
    static const std::size_t MsgMaxLen = Base::doMaxLength();
    static_assert(MsgMinLen == 4U, "Unexpected min serialisation length");
    static_assert(MsgMaxLen == 4U, "Unexpected max serialisation length");
    
    /// @brief Name of the message.
    static const char* doName()
    {
        return "CFG-RST";
    }
    
    
};

} // namespace message

} // namespace ublox


