/// @file
/// @brief Contains definition of <b>"ESF-STATUS"</b> message and its fields.

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
#include "ublox/field/Itow.h"
#include "ublox/field/Res2.h"
#include "ublox/field/Res7.h"

namespace ublox
{

namespace message
{

/// @brief Fields of @ref EsfStatus.
/// @tparam TOpt Extra options
/// @see @ref EsfStatus
/// @headerfile "ublox/message/EsfStatus.h"
template <typename TOpt = ublox::DefaultOptions>
struct EsfStatusFields
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
            comms::option::DefaultNumValue<2>,
            comms::option::ValidNumValueRange<0, 2>
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "version";
        }
        
    };
    
    /// @brief Definition of <b>"reserved1"</b> field.
    struct Reserved1 : public
        ublox::field::Res7<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "reserved1";
        }
        
    };
    
    /// @brief Values enumerator for @ref FusionMode field.
    enum class FusionModeVal : std::uint8_t
    {
        Initialization = 0, ///< value @b Initialization
        Fusion = 1, ///< value @b Fusion
        Suspended = 2, ///< value @b Suspended
        Disabled = 3, ///< value @b Disabled
        
    };
    
    /// @brief Definition of <b>"fusionMode"</b> field.
    struct FusionMode : public
        comms::field::EnumValue<
            ublox::field::FieldBase<>,
            FusionModeVal,
            comms::option::ValidNumValueRange<0, 3>
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "fusionMode";
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
    
    /// @brief Definition of <b>"numSens"</b> field.
    struct NumSens : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "numSens";
        }
        
    };
    
    /// @brief Scope for all the member fields of @ref List list.
    struct ListMembers
    {
        /// @brief Scope for all the member fields of @ref Element bitfield.
        struct ElementMembers
        {
            /// @brief Scope for all the member fields of @ref SensStatus1 bitfield.
            struct SensStatus1Members
            {
                /// @brief Definition of <b>"type"</b> field.
                struct Type : public
                    comms::field::IntValue<
                        ublox::field::FieldBase<>,
                        std::uint8_t,
                        comms::option::FixedBitLength<6U>
                    >
                {
                    /// @brief Name of the field.
                    static const char* name()
                    {
                        return "type";
                    }
                    
                };
                
                /// @brief Definition of <b>""</b> field.
                class Bits : public
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
                    ///      @li @b BitIdx_used, @b getBitValue_used() and @b setBitValue_used().
                    ///      @li @b BitIdx_ready, @b getBitValue_ready() and @b setBitValue_ready().
                    COMMS_BITMASK_BITS_SEQ(
                        used,
                        ready
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
                       Type,
                       Bits
                    >;
            };
            
            /// @brief Definition of <b>"sensStatus1"</b> field.
            class SensStatus1 : public
                comms::field::Bitfield<
                    ublox::field::FieldBase<>,
                    typename SensStatus1Members::All
                >
            {
                using Base = 
                    comms::field::Bitfield<
                        ublox::field::FieldBase<>,
                        typename SensStatus1Members::All
                    >;
            public:
                /// @brief Allow access to internal fields.
                /// @details See definition of @b COMMS_FIELD_MEMBERS_ACCESS macro
                ///     related to @b comms::field::Bitfield class from COMMS library
                ///     for details.
                ///
                ///      The generated access functions are:
                ///     @li @b field_type() - for SensStatus1Members::Type member field.
                ///     @li @b field_bits() - for SensStatus1Members::Bits member field.
                COMMS_FIELD_MEMBERS_ACCESS(
                    type,
                    bits
                );
                
                /// @brief Name of the field.
                static const char* name()
                {
                    return "sensStatus1";
                }
                
            };
            
            /// @brief Scope for all the member fields of @ref SensStatus2 bitfield.
            struct SensStatus2Members
            {
                /// @brief Values enumerator for @ref CalibStatus field.
                enum class CalibStatusVal : std::uint8_t
                {
                    NotCalibrated = 0, ///< value @b NotCalibrated
                    Calibrating = 1, ///< value @b Calibrating
                    Calibrated = 2, ///< value @b Calibrated
                    Calibrated2 = 3, ///< value @b Calibrated2
                    
                };
                
                /// @brief Definition of <b>"calibStatus"</b> field.
                struct CalibStatus : public
                    comms::field::EnumValue<
                        ublox::field::FieldBase<>,
                        CalibStatusVal,
                        comms::option::FixedBitLength<2U>,
                        comms::option::ValidNumValueRange<0, 3>
                    >
                {
                    /// @brief Name of the field.
                    static const char* name()
                    {
                        return "calibStatus";
                    }
                    
                };
                
                /// @brief Values enumerator for @ref TimeStatus field.
                enum class TimeStatusVal : std::uint8_t
                {
                    NoData = 0, ///< value @b NoData
                    FirstByte = 1, ///< value @b FirstByte
                    EventInput = 2, ///< value @b EventInput
                    TimeTag = 3, ///< value @b TimeTag
                    
                };
                
                /// @brief Definition of <b>"timeStatus"</b> field.
                struct TimeStatus : public
                    comms::field::EnumValue<
                        ublox::field::FieldBase<>,
                        TimeStatusVal,
                        comms::option::FixedBitLength<2U>,
                        comms::option::ValidNumValueRange<0, 3>
                    >
                {
                    /// @brief Name of the field.
                    static const char* name()
                    {
                        return "timeStatus";
                    }
                    
                };
                
                /// @brief Definition of <b>"reserved"</b> field.
                /// @details
                ///     Reserved field with 1 byte length
                struct Reserved : public
                    comms::field::IntValue<
                        ublox::field::FieldBase<>,
                        std::uint8_t,
                        comms::option::FixedBitLength<4U>,
                        comms::option::ValidNumValue<0>
                    >
                {
                    /// @brief Name of the field.
                    static const char* name()
                    {
                        return "reserved";
                    }
                    
                };
                
                /// @brief All members bundled in @b std::tuple.
                using All =
                    std::tuple<
                       CalibStatus,
                       TimeStatus,
                       Reserved
                    >;
            };
            
            /// @brief Definition of <b>"sensStatus2"</b> field.
            class SensStatus2 : public
                comms::field::Bitfield<
                    ublox::field::FieldBase<>,
                    typename SensStatus2Members::All
                >
            {
                using Base = 
                    comms::field::Bitfield<
                        ublox::field::FieldBase<>,
                        typename SensStatus2Members::All
                    >;
            public:
                /// @brief Allow access to internal fields.
                /// @details See definition of @b COMMS_FIELD_MEMBERS_ACCESS macro
                ///     related to @b comms::field::Bitfield class from COMMS library
                ///     for details.
                ///
                ///      The generated access functions are:
                ///     @li @b field_calibStatus() - for SensStatus2Members::CalibStatus member field.
                ///     @li @b field_timeStatus() - for SensStatus2Members::TimeStatus member field.
                ///     @li @b field_reserved() - for SensStatus2Members::Reserved member field.
                COMMS_FIELD_MEMBERS_ACCESS(
                    calibStatus,
                    timeStatus,
                    reserved
                );
                
                /// @brief Name of the field.
                static const char* name()
                {
                    return "sensStatus2";
                }
                
            };
            
            /// @brief Definition of <b>"freq"</b> field.
            struct Freq : public
                comms::field::IntValue<
                    ublox::field::FieldBase<>,
                    std::uint8_t,
                    comms::option::UnitsHertz
                >
            {
                /// @brief Name of the field.
                static const char* name()
                {
                    return "freq";
                }
                
            };
            
            /// @brief Definition of <b>"faults"</b> field.
            class Faults : public
                comms::field::BitmaskValue<
                    ublox::field::FieldBase<>,
                    comms::option::FixedLength<1U>,
                    comms::option::BitmaskReservedBits<0xF0U, 0x0U>
                >
            {
                using Base = 
                    comms::field::BitmaskValue<
                        ublox::field::FieldBase<>,
                        comms::option::FixedLength<1U>,
                        comms::option::BitmaskReservedBits<0xF0U, 0x0U>
                    >;
            public:
                /// @brief Provides names and generates access functions for internal bits.
                /// @details See definition of @b COMMS_BITMASK_BITS_SEQ macro
                ///     related to @b comms::field::BitmaskValue class from COMMS library
                ///     for details.
                ///
                ///      The generated enum values and functions are:
                ///      @li @b BitIdx_badMeas, @b getBitValue_badMeas() and @b setBitValue_badMeas().
                ///      @li @b BitIdx_badTTag, @b getBitValue_badTTag() and @b setBitValue_badTTag().
                ///      @li @b BitIdx_missingMeas, @b getBitValue_missingMeas() and @b setBitValue_missingMeas().
                ///      @li @b BitIdx_noisyMeas, @b getBitValue_noisyMeas() and @b setBitValue_noisyMeas().
                COMMS_BITMASK_BITS_SEQ(
                    badMeas,
                    badTTag,
                    missingMeas,
                    noisyMeas
                );
                
                /// @brief Name of the field.
                static const char* name()
                {
                    return "faults";
                }
                
            };
            
            /// @brief All members bundled in @b std::tuple.
            using All =
                std::tuple<
                   SensStatus1,
                   SensStatus2,
                   Freq,
                   Faults
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
            ///     @li @b field_sensStatus1() - for ElementMembers::SensStatus1 member field.
            ///     @li @b field_sensStatus2() - for ElementMembers::SensStatus2 member field.
            ///     @li @b field_freq() - for ElementMembers::Freq member field.
            ///     @li @b field_faults() - for ElementMembers::Faults member field.
            COMMS_FIELD_MEMBERS_ACCESS(
                sensStatus1,
                sensStatus2,
                freq,
                faults
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
            typename TOpt::message::EsfStatusFields::List,
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
        Reserved1,
        FusionMode,
        Reserved2,
        NumSens,
        List
    >;
};

/// @brief Definition of <b>"ESF-STATUS"</b> message class.
/// @details
///     See @ref EsfStatusFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/EsfStatus.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class EsfStatus : public
    comms::MessageBase<
        TMsgBase,
        typename TOpt::message::EsfStatus,
        comms::option::StaticNumIdImpl<ublox::MsgId_EsfStatus>,
        comms::option::FieldsImpl<typename EsfStatusFields<TOpt>::All>,
        comms::option::MsgType<EsfStatus<TMsgBase, TOpt> >,
        comms::option::HasName,
        comms::option::HasCustomRefresh
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            typename TOpt::message::EsfStatus,
            comms::option::StaticNumIdImpl<ublox::MsgId_EsfStatus>,
            comms::option::FieldsImpl<typename EsfStatusFields<TOpt>::All>,
            comms::option::MsgType<EsfStatus<TMsgBase, TOpt> >,
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
    ///     @li @b field_itow() for @ref EsfStatusFields::Itow field.
    ///     @li @b field_version() for @ref EsfStatusFields::Version field.
    ///     @li @b field_reserved1() for @ref EsfStatusFields::Reserved1 field.
    ///     @li @b field_fusionMode() for @ref EsfStatusFields::FusionMode field.
    ///     @li @b field_reserved2() for @ref EsfStatusFields::Reserved2 field.
    ///     @li @b field_numSens() for @ref EsfStatusFields::NumSens field.
    ///     @li @b field_list() for @ref EsfStatusFields::List field.
    COMMS_MSG_FIELDS_ACCESS(
        itow,
        version,
        reserved1,
        fusionMode,
        reserved2,
        numSens,
        list
    );
    
    // Compile time check for serialisation length.
    static const std::size_t MsgMinLen = Base::doMinLength();
    static_assert(MsgMinLen == 16U, "Unexpected min serialisation length");
    
    /// @brief Name of the message.
    static const char* doName()
    {
        return "ESF-STATUS";
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
            static_cast<std::size_t>(field_numSens().value()));
        
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
            auto expectedValue = static_cast<std::size_t>(field_numSens().value());
            auto realValue = field_list().value().size();
            if (expectedValue != realValue) {
                using PrefixValueType = typename std::decay<decltype(field_numSens().value())>::type;
                field_numSens().value() = static_cast<PrefixValueType>(realValue);
                updated = true;
            }
        } while (false);
        
        return updated;
        
    }
    
};

} // namespace message

} // namespace ublox


