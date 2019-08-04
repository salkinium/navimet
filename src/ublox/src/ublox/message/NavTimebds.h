/// @file
/// @brief Contains definition of <b>"NAV-TIMEBDS"</b> message and its fields.

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

namespace ublox
{

namespace message
{

/// @brief Fields of @ref NavTimebds.
/// @tparam TOpt Extra options
/// @see @ref NavTimebds
/// @headerfile "ublox/message/NavTimebds.h"
template <typename TOpt = ublox::DefaultOptions>
struct NavTimebdsFields
{
    /// @brief Definition of <b>"iTOW"</b> field.
    using Itow =
        ublox::field::Itow<
           TOpt
       >;
    
    /// @brief Definition of <b>"SOW"</b> field.
    struct SOW : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint32_t,
            comms::option::UnitsSeconds
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "SOW";
        }
        
    };
    
    /// @brief Definition of <b>"fSOW"</b> field.
    struct FSOW : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::int32_t,
            comms::option::UnitsNanoseconds,
            comms::option::ValidNumValueRange<-500000000L, 500000000L>
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "fSOW";
        }
        
    };
    
    /// @brief Definition of <b>"week"</b> field.
    struct Week : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::int16_t,
            comms::option::UnitsWeeks
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "week";
        }
        
    };
    
    /// @brief Definition of <b>"leapS"</b> field.
    struct LeapS : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::int8_t,
            comms::option::UnitsSeconds
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "leapS";
        }
        
    };
    
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
        ///      @li @b BitIdx_sowValid, @b getBitValue_sowValid() and @b setBitValue_sowValid().
        ///      @li @b BitIdx_weekValid, @b getBitValue_weekValid() and @b setBitValue_weekValid().
        ///      @li @b BitIdx_leapSValid, @b getBitValue_leapSValid() and @b setBitValue_leapSValid().
        COMMS_BITMASK_BITS_SEQ(
            sowValid,
            weekValid,
            leapSValid
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
    
    /// @brief All the fields bundled in std::tuple.
    using All = std::tuple<
        Itow,
        SOW,
        FSOW,
        Week,
        LeapS,
        Valid,
        TAcc
    >;
};

/// @brief Definition of <b>"NAV-TIMEBDS"</b> message class.
/// @details
///     See @ref NavTimebdsFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/NavTimebds.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class NavTimebds : public
    comms::MessageBase<
        TMsgBase,
        typename TOpt::message::NavTimebds,
        comms::option::StaticNumIdImpl<ublox::MsgId_NavTimebds>,
        comms::option::FieldsImpl<typename NavTimebdsFields<TOpt>::All>,
        comms::option::MsgType<NavTimebds<TMsgBase, TOpt> >,
        comms::option::HasName
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            typename TOpt::message::NavTimebds,
            comms::option::StaticNumIdImpl<ublox::MsgId_NavTimebds>,
            comms::option::FieldsImpl<typename NavTimebdsFields<TOpt>::All>,
            comms::option::MsgType<NavTimebds<TMsgBase, TOpt> >,
            comms::option::HasName
        >;

public:
    /// @brief Allow access to internal fields.
    /// @details See definition of @b COMMS_MSG_FIELDS_ACCESS macro
    ///     related to @b comms::MessageBase class from COMMS library
    ///     for details.
    ///
    ///     The generated functions are:
    ///     @li @b field_itow() for @ref NavTimebdsFields::Itow field.
    ///     @li @b field_sOW() for @ref NavTimebdsFields::SOW field.
    ///     @li @b field_fSOW() for @ref NavTimebdsFields::FSOW field.
    ///     @li @b field_week() for @ref NavTimebdsFields::Week field.
    ///     @li @b field_leapS() for @ref NavTimebdsFields::LeapS field.
    ///     @li @b field_valid() for @ref NavTimebdsFields::Valid field.
    ///     @li @b field_tAcc() for @ref NavTimebdsFields::TAcc field.
    COMMS_MSG_FIELDS_ACCESS(
        itow,
        sOW,
        fSOW,
        week,
        leapS,
        valid,
        tAcc
    );
    
    // Compile time check for serialisation length.
    static const std::size_t MsgMinLen = Base::doMinLength();
    static const std::size_t MsgMaxLen = Base::doMaxLength();
    static_assert(MsgMinLen == 20U, "Unexpected min serialisation length");
    static_assert(MsgMaxLen == 20U, "Unexpected max serialisation length");
    
    /// @brief Name of the message.
    static const char* doName()
    {
        return "NAV-TIMEBDS";
    }
    
    
};

} // namespace message

} // namespace ublox


