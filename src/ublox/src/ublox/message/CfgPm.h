/// @file
/// @brief Contains definition of <b>"CFG-PM"</b> message and its fields.

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
#include "ublox/field/FieldBase.h"
#include "ublox/field/Res1.h"

namespace ublox
{

namespace message
{

/// @brief Fields of @ref CfgPm.
/// @tparam TOpt Extra options
/// @see @ref CfgPm
/// @headerfile "ublox/message/CfgPm.h"
template <typename TOpt = ublox::DefaultOptions>
struct CfgPmFields
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
    
    /// @brief Definition of <b>"res1"</b> field.
    using Res1 =
        ublox::field::Res1<
           TOpt
       >;
    
    /// @brief Definition of <b>"res2"</b> field.
    struct Res2 : public
        ublox::field::Res1<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "res2";
        }
        
    };
    
    /// @brief Definition of <b>"res3"</b> field.
    struct Res3 : public
        ublox::field::Res1<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "res3";
        }
        
    };
    
    /// @brief Scope for all the member fields of @ref Flags bitfield.
    struct FlagsMembers
    {
        /// @brief Definition of <b>"res1"</b> field.
        /// @details
        ///     Reserved field with 1 byte length
        struct Res1 : public
            comms::field::IntValue<
                ublox::field::FieldBase<>,
                std::uint8_t,
                comms::option::FixedBitLength<2U>,
                comms::option::ValidNumValue<0>
            >
        {
            /// @brief Name of the field.
            static const char* name()
            {
                return "res1";
            }
            
        };
        
        /// @brief Definition of <b>"internal"</b> field.
        struct Internal : public
            comms::field::IntValue<
                ublox::field::FieldBase<>,
                std::uint8_t,
                comms::option::DefaultNumValue<1>,
                comms::option::FixedBitLength<2U>,
                comms::option::ValidNumValue<1>
            >
        {
            /// @brief Name of the field.
            static const char* name()
            {
                return "internal";
            }
            
        };
        
        /// @brief Definition of <b>"bitsMid"</b> field.
        class BitsMid : public
            comms::field::BitmaskValue<
                ublox::field::FieldBase<>,
                comms::option::FixedBitLength<4U>,
                comms::option::BitmaskReservedBits<0x8U, 0x0U>
            >
        {
            using Base = 
                comms::field::BitmaskValue<
                    ublox::field::FieldBase<>,
                    comms::option::FixedBitLength<4U>,
                    comms::option::BitmaskReservedBits<0x8U, 0x0U>
                >;
        public:
            /// @brief Provides names and generates access functions for internal bits.
            /// @details See definition of @b COMMS_BITMASK_BITS_SEQ macro
            ///     related to @b comms::field::BitmaskValue class from COMMS library
            ///     for details.
            ///
            ///      The generated enum values and functions are:
            ///      @li @b BitIdx_extintSelect, @b getBitValue_extintSelect() and @b setBitValue_extintSelect().
            ///      @li @b BitIdx_extintWake, @b getBitValue_extintWake() and @b setBitValue_extintWake().
            ///      @li @b BitIdx_extintBackup, @b getBitValue_extintBackup() and @b setBitValue_extintBackup().
            COMMS_BITMASK_BITS_SEQ(
                extintSelect,
                extintWake,
                extintBackup
            );
            
            /// @brief Name of the field.
            static const char* name()
            {
                return "bitsMid";
            }
            
        };
        
        /// @brief Values enumerator for @ref LimitPeakCurr field.
        enum class LimitPeakCurrVal : std::uint8_t
        {
            Disabled = 0, ///< value @b Disabled
            Enabled = 1, ///< value @b Enabled
            
        };
        
        /// @brief Definition of <b>"limitPeakCurr"</b> field.
        struct LimitPeakCurr : public
            comms::field::EnumValue<
                ublox::field::FieldBase<>,
                LimitPeakCurrVal,
                comms::option::FixedBitLength<2U>,
                comms::option::ValidNumValueRange<0, 1>
            >
        {
            /// @brief Name of the field.
            static const char* name()
            {
                return "limitPeakCurr";
            }
            
        };
        
        /// @brief Definition of <b>"bitsHigh"</b> field.
        class BitsHigh : public
            comms::field::BitmaskValue<
                ublox::field::FieldBase<>,
                comms::option::FixedBitLength<22U>,
                comms::option::BitmaskReservedBits<0x3FFFF8UL, 0x0U>
            >
        {
            using Base = 
                comms::field::BitmaskValue<
                    ublox::field::FieldBase<>,
                    comms::option::FixedBitLength<22U>,
                    comms::option::BitmaskReservedBits<0x3FFFF8UL, 0x0U>
                >;
        public:
            /// @brief Provides names and generates access functions for internal bits.
            /// @details See definition of @b COMMS_BITMASK_BITS_SEQ macro
            ///     related to @b comms::field::BitmaskValue class from COMMS library
            ///     for details.
            ///
            ///      The generated enum values and functions are:
            ///      @li @b BitIdx_WaitTimeFix, @b getBitValue_WaitTimeFix() and @b setBitValue_WaitTimeFix().
            ///      @li @b BitIdx_updateRTC, @b getBitValue_updateRTC() and @b setBitValue_updateRTC().
            ///      @li @b BitIdx_updateEPH, @b getBitValue_updateEPH() and @b setBitValue_updateEPH().
            COMMS_BITMASK_BITS_SEQ(
                WaitTimeFix,
                updateRTC,
                updateEPH
            );
            
            /// @brief Name of the field.
            static const char* name()
            {
                return "bitsHigh";
            }
            
        };
        
        /// @brief All members bundled in @b std::tuple.
        using All =
            std::tuple<
               Res1,
               Internal,
               BitsMid,
               LimitPeakCurr,
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
        ///     @li @b field_res1() - for FlagsMembers::Res1 member field.
        ///     @li @b field_internal() - for FlagsMembers::Internal member field.
        ///     @li @b field_bitsMid() - for FlagsMembers::BitsMid member field.
        ///     @li @b field_limitPeakCurr() - for FlagsMembers::LimitPeakCurr member field.
        ///     @li @b field_bitsHigh() - for FlagsMembers::BitsHigh member field.
        COMMS_FIELD_MEMBERS_ACCESS(
            res1,
            internal,
            bitsMid,
            limitPeakCurr,
            bitsHigh
        );
        
        /// @brief Name of the field.
        static const char* name()
        {
            return "flags";
        }
        
    };
    
    /// @brief Definition of <b>"updatePeriod"</b> field.
    struct UpdatePeriod : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint32_t,
            comms::option::UnitsMilliseconds
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "updatePeriod";
        }
        
    };
    
    /// @brief Definition of <b>"searchPeriod"</b> field.
    struct SearchPeriod : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint32_t,
            comms::option::UnitsMilliseconds
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "searchPeriod";
        }
        
    };
    
    /// @brief Definition of <b>"gridOffset"</b> field.
    struct GridOffset : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint32_t,
            comms::option::UnitsMilliseconds
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "gridOffset";
        }
        
    };
    
    /// @brief Definition of <b>"onTime"</b> field.
    struct OnTime : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint16_t,
            comms::option::UnitsSeconds
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "onTime";
        }
        
    };
    
    /// @brief Definition of <b>"minAcqTime"</b> field.
    struct MinAcqTime : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint16_t,
            comms::option::UnitsSeconds
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "minAcqTime";
        }
        
    };
    
    /// @brief All the fields bundled in std::tuple.
    using All = std::tuple<
        Version,
        Res1,
        Res2,
        Res3,
        Flags,
        UpdatePeriod,
        SearchPeriod,
        GridOffset,
        OnTime,
        MinAcqTime
    >;
};

/// @brief Definition of <b>"CFG-PM"</b> message class.
/// @details
///     See @ref CfgPmFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/CfgPm.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class CfgPm : public
    comms::MessageBase<
        TMsgBase,
        comms::option::StaticNumIdImpl<ublox::MsgId_CfgPm>,
        comms::option::FieldsImpl<typename CfgPmFields<TOpt>::All>,
        comms::option::MsgType<CfgPm<TMsgBase, TOpt> >,
        comms::option::HasName
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            comms::option::StaticNumIdImpl<ublox::MsgId_CfgPm>,
            comms::option::FieldsImpl<typename CfgPmFields<TOpt>::All>,
            comms::option::MsgType<CfgPm<TMsgBase, TOpt> >,
            comms::option::HasName
        >;

public:
    /// @brief Allow access to internal fields.
    /// @details See definition of @b COMMS_MSG_FIELDS_ACCESS macro
    ///     related to @b comms::MessageBase class from COMMS library
    ///     for details.
    ///
    ///     The generated functions are:
    ///     @li @b field_version() for @ref CfgPmFields::Version field.
    ///     @li @b field_res1() for @ref CfgPmFields::Res1 field.
    ///     @li @b field_res2() for @ref CfgPmFields::Res2 field.
    ///     @li @b field_res3() for @ref CfgPmFields::Res3 field.
    ///     @li @b field_flags() for @ref CfgPmFields::Flags field.
    ///     @li @b field_updatePeriod() for @ref CfgPmFields::UpdatePeriod field.
    ///     @li @b field_searchPeriod() for @ref CfgPmFields::SearchPeriod field.
    ///     @li @b field_gridOffset() for @ref CfgPmFields::GridOffset field.
    ///     @li @b field_onTime() for @ref CfgPmFields::OnTime field.
    ///     @li @b field_minAcqTime() for @ref CfgPmFields::MinAcqTime field.
    COMMS_MSG_FIELDS_ACCESS(
        version,
        res1,
        res2,
        res3,
        flags,
        updatePeriod,
        searchPeriod,
        gridOffset,
        onTime,
        minAcqTime
    );
    
    // Compile time check for serialisation length.
    static const std::size_t MsgMinLen = Base::doMinLength();
    static const std::size_t MsgMaxLen = Base::doMaxLength();
    static_assert(MsgMinLen == 24U, "Unexpected min serialisation length");
    static_assert(MsgMaxLen == 24U, "Unexpected max serialisation length");
    
    /// @brief Name of the message.
    static const char* doName()
    {
        return "CFG-PM";
    }
    
    
};

} // namespace message

} // namespace ublox


