/// @file
/// @brief Contains definition of <b>"RXM-SFRB"</b> message and its fields.

#pragma once

#include <cstdint>
#include <tuple>
#include "comms/MessageBase.h"
#include "comms/field/ArrayList.h"
#include "comms/field/IntValue.h"
#include "comms/options.h"
#include "ublox/DefaultOptions.h"
#include "ublox/MsgId.h"
#include "ublox/field/FieldBase.h"

namespace ublox
{

namespace message
{

/// @brief Fields of @ref RxmSfrb.
/// @tparam TOpt Extra options
/// @see @ref RxmSfrb
/// @headerfile "ublox/message/RxmSfrb.h"
template <typename TOpt = ublox::DefaultOptions>
struct RxmSfrbFields
{
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
    
    /// @brief Scope for all the member fields of @ref Dwrd list.
    struct DwrdMembers
    {
        /// @brief Definition of <b>"element"</b> field.
        struct Element : public
            comms::field::IntValue<
                ublox::field::FieldBase<>,
                std::uint32_t
            >
        {
            /// @brief Name of the field.
            static const char* name()
            {
                return "element";
            }
            
        };
        
    };
    
    /// @brief Definition of <b>"dwrd"</b> field.
    struct Dwrd : public
        comms::field::ArrayList<
            ublox::field::FieldBase<>,
            typename DwrdMembers::Element,
            typename TOpt::message::RxmSfrbFields::Dwrd,
            comms::option::SequenceFixedSize<10U>
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "dwrd";
        }
        
    };
    
    /// @brief All the fields bundled in std::tuple.
    using All = std::tuple<
        Cno,
        Svid,
        Dwrd
    >;
};

/// @brief Definition of <b>"RXM-SFRB"</b> message class.
/// @details
///     See @ref RxmSfrbFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/RxmSfrb.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class RxmSfrb : public
    comms::MessageBase<
        TMsgBase,
        typename TOpt::message::RxmSfrb,
        comms::option::StaticNumIdImpl<ublox::MsgId_RxmSfrb>,
        comms::option::FieldsImpl<typename RxmSfrbFields<TOpt>::All>,
        comms::option::MsgType<RxmSfrb<TMsgBase, TOpt> >,
        comms::option::HasName
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            typename TOpt::message::RxmSfrb,
            comms::option::StaticNumIdImpl<ublox::MsgId_RxmSfrb>,
            comms::option::FieldsImpl<typename RxmSfrbFields<TOpt>::All>,
            comms::option::MsgType<RxmSfrb<TMsgBase, TOpt> >,
            comms::option::HasName
        >;

public:
    /// @brief Allow access to internal fields.
    /// @details See definition of @b COMMS_MSG_FIELDS_ACCESS macro
    ///     related to @b comms::MessageBase class from COMMS library
    ///     for details.
    ///
    ///     The generated functions are:
    ///     @li @b field_cno() for @ref RxmSfrbFields::Cno field.
    ///     @li @b field_svid() for @ref RxmSfrbFields::Svid field.
    ///     @li @b field_dwrd() for @ref RxmSfrbFields::Dwrd field.
    COMMS_MSG_FIELDS_ACCESS(
        cno,
        svid,
        dwrd
    );
    
    // Compile time check for serialisation length.
    static const std::size_t MsgMinLen = Base::doMinLength();
    static const std::size_t MsgMaxLen = Base::doMaxLength();
    static_assert(MsgMinLen == 42U, "Unexpected min serialisation length");
    static_assert(MsgMaxLen == 42U, "Unexpected max serialisation length");
    
    /// @brief Name of the message.
    static const char* doName()
    {
        return "RXM-SFRB";
    }
    
    
};

} // namespace message

} // namespace ublox


