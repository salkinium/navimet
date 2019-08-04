/// @file
/// @brief Contains definition of <b>"RXM-RTCM"</b> message and its fields.

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
#include "ublox/field/Res2.h"

namespace ublox
{

namespace message
{

/// @brief Fields of @ref RxmRtcm.
/// @tparam TOpt Extra options
/// @see @ref RxmRtcm
/// @headerfile "ublox/message/RxmRtcm.h"
template <typename TOpt = ublox::DefaultOptions>
struct RxmRtcmFields
{
    /// @brief Definition of <b>"version"</b> field.
    struct Version : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t,
            comms::option::DefaultNumValue<2>,
            comms::option::ValidNumValue<2>
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "version";
        }
        
    };
    
    /// @brief Definition of <b>"flags"</b> field.
    class Flags : public
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
        ///      @li @b BitIdx_crcFailed, @b getBitValue_crcFailed() and @b setBitValue_crcFailed().
        COMMS_BITMASK_BITS_SEQ(
            crcFailed
        );
        
        /// @brief Name of the field.
        static const char* name()
        {
            return "flags";
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
    
    /// @brief Definition of <b>"refStation"</b> field.
    struct RefStation : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint16_t
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "refStation";
        }
        
    };
    
    /// @brief Definition of <b>"msgType"</b> field.
    struct MsgType : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint16_t
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "msgType";
        }
        
    };
    
    /// @brief All the fields bundled in std::tuple.
    using All = std::tuple<
        Version,
        Flags,
        Reserved1,
        RefStation,
        MsgType
    >;
};

/// @brief Definition of <b>"RXM-RTCM"</b> message class.
/// @details
///     See @ref RxmRtcmFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/RxmRtcm.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class RxmRtcm : public
    comms::MessageBase<
        TMsgBase,
        typename TOpt::message::RxmRtcm,
        comms::option::StaticNumIdImpl<ublox::MsgId_RxmRtcm>,
        comms::option::FieldsImpl<typename RxmRtcmFields<TOpt>::All>,
        comms::option::MsgType<RxmRtcm<TMsgBase, TOpt> >,
        comms::option::HasName
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            typename TOpt::message::RxmRtcm,
            comms::option::StaticNumIdImpl<ublox::MsgId_RxmRtcm>,
            comms::option::FieldsImpl<typename RxmRtcmFields<TOpt>::All>,
            comms::option::MsgType<RxmRtcm<TMsgBase, TOpt> >,
            comms::option::HasName
        >;

public:
    /// @brief Allow access to internal fields.
    /// @details See definition of @b COMMS_MSG_FIELDS_ACCESS macro
    ///     related to @b comms::MessageBase class from COMMS library
    ///     for details.
    ///
    ///     The generated functions are:
    ///     @li @b field_version() for @ref RxmRtcmFields::Version field.
    ///     @li @b field_flags() for @ref RxmRtcmFields::Flags field.
    ///     @li @b field_reserved1() for @ref RxmRtcmFields::Reserved1 field.
    ///     @li @b field_refStation() for @ref RxmRtcmFields::RefStation field.
    ///     @li @b field_msgType() for @ref RxmRtcmFields::MsgType field.
    COMMS_MSG_FIELDS_ACCESS(
        version,
        flags,
        reserved1,
        refStation,
        msgType
    );
    
    // Compile time check for serialisation length.
    static const std::size_t MsgMinLen = Base::doMinLength();
    static const std::size_t MsgMaxLen = Base::doMaxLength();
    static_assert(MsgMinLen == 8U, "Unexpected min serialisation length");
    static_assert(MsgMaxLen == 8U, "Unexpected max serialisation length");
    
    /// @brief Name of the message.
    static const char* doName()
    {
        return "RXM-RTCM";
    }
    
    
};

} // namespace message

} // namespace ublox


