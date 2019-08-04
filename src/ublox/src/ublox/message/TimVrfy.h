/// @file
/// @brief Contains definition of <b>"TIM-VRFY"</b> message and its fields.

#pragma once

#include <cstdint>
#include <tuple>
#include "comms/MessageBase.h"
#include "comms/field/Bitfield.h"
#include "comms/field/EnumValue.h"
#include "comms/field/IntValue.h"
#include "comms/options.h"
#include "ublox/DefaultOptions.h"
#include "ublox/MsgId.h"
#include "ublox/field/FieldBase.h"
#include "ublox/field/Itow.h"
#include "ublox/field/Res1.h"

namespace ublox
{

namespace message
{

/// @brief Fields of @ref TimVrfy.
/// @tparam TOpt Extra options
/// @see @ref TimVrfy
/// @headerfile "ublox/message/TimVrfy.h"
template <typename TOpt = ublox::DefaultOptions>
struct TimVrfyFields
{
    /// @brief Definition of <b>"iTOW"</b> field.
    using Itow =
        ublox::field::Itow<
           TOpt
       >;
    
    /// @brief Definition of <b>"frac"</b> field.
    struct Frac : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::int32_t,
            comms::option::UnitsNanoseconds
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "frac";
        }
        
    };
    
    /// @brief Definition of <b>"deltaMs"</b> field.
    struct DeltaMs : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::int32_t,
            comms::option::UnitsMilliseconds
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "deltaMs";
        }
        
    };
    
    /// @brief Definition of <b>"deltaNs"</b> field.
    struct DeltaNs : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::int32_t,
            comms::option::UnitsNanoseconds
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "deltaNs";
        }
        
    };
    
    /// @brief Definition of <b>"wno"</b> field.
    struct Wno : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint16_t,
            comms::option::UnitsWeeks
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "wno";
        }
        
    };
    
    /// @brief Scope for all the member fields of @ref Flags bitfield.
    struct FlagsMembers
    {
        /// @brief Values enumerator for @ref Src field.
        enum class SrcVal : std::uint8_t
        {
            NoAiding = 0, ///< value @b NoAiding
            RTC = 1, ///< value @b RTC
            AidIni = 2, ///< value @b AidIni
            
        };
        
        /// @brief Definition of <b>"src"</b> field.
        struct Src : public
            comms::field::EnumValue<
                ublox::field::FieldBase<>,
                SrcVal,
                comms::option::FixedBitLength<3U>,
                comms::option::ValidNumValueRange<0, 2>
            >
        {
            /// @brief Name of the field.
            static const char* name()
            {
                return "src";
            }
            
        };
        
        /// @brief Definition of <b>"reserved"</b> field.
        /// @details
        ///     Reserved field with 1 byte length
        struct Reserved : public
            comms::field::IntValue<
                ublox::field::FieldBase<>,
                std::uint8_t,
                comms::option::FixedBitLength<5U>,
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
               Src,
               Reserved
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
        ///     @li @b field_src() - for FlagsMembers::Src member field.
        ///     @li @b field_reserved() - for FlagsMembers::Reserved member field.
        COMMS_FIELD_MEMBERS_ACCESS(
            src,
            reserved
        );
        
        /// @brief Name of the field.
        static const char* name()
        {
            return "flags";
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
        Itow,
        Frac,
        DeltaMs,
        DeltaNs,
        Wno,
        Flags,
        Reserved1
    >;
};

/// @brief Definition of <b>"TIM-VRFY"</b> message class.
/// @details
///     See @ref TimVrfyFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/TimVrfy.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class TimVrfy : public
    comms::MessageBase<
        TMsgBase,
        typename TOpt::message::TimVrfy,
        comms::option::StaticNumIdImpl<ublox::MsgId_TimVrfy>,
        comms::option::FieldsImpl<typename TimVrfyFields<TOpt>::All>,
        comms::option::MsgType<TimVrfy<TMsgBase, TOpt> >,
        comms::option::HasName
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            typename TOpt::message::TimVrfy,
            comms::option::StaticNumIdImpl<ublox::MsgId_TimVrfy>,
            comms::option::FieldsImpl<typename TimVrfyFields<TOpt>::All>,
            comms::option::MsgType<TimVrfy<TMsgBase, TOpt> >,
            comms::option::HasName
        >;

public:
    /// @brief Allow access to internal fields.
    /// @details See definition of @b COMMS_MSG_FIELDS_ACCESS macro
    ///     related to @b comms::MessageBase class from COMMS library
    ///     for details.
    ///
    ///     The generated functions are:
    ///     @li @b field_itow() for @ref TimVrfyFields::Itow field.
    ///     @li @b field_frac() for @ref TimVrfyFields::Frac field.
    ///     @li @b field_deltaMs() for @ref TimVrfyFields::DeltaMs field.
    ///     @li @b field_deltaNs() for @ref TimVrfyFields::DeltaNs field.
    ///     @li @b field_wno() for @ref TimVrfyFields::Wno field.
    ///     @li @b field_flags() for @ref TimVrfyFields::Flags field.
    ///     @li @b field_reserved1() for @ref TimVrfyFields::Reserved1 field.
    COMMS_MSG_FIELDS_ACCESS(
        itow,
        frac,
        deltaMs,
        deltaNs,
        wno,
        flags,
        reserved1
    );
    
    // Compile time check for serialisation length.
    static const std::size_t MsgMinLen = Base::doMinLength();
    static const std::size_t MsgMaxLen = Base::doMaxLength();
    static_assert(MsgMinLen == 20U, "Unexpected min serialisation length");
    static_assert(MsgMaxLen == 20U, "Unexpected max serialisation length");
    
    /// @brief Name of the message.
    static const char* doName()
    {
        return "TIM-VRFY";
    }
    
    
};

} // namespace message

} // namespace ublox


