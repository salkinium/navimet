/// @file
/// @brief Contains definition of <b>"MGA-BDS-POS_LLH"</b> message and its fields.

#pragma once

#include <cstdint>
#include <tuple>
#include "comms/MessageBase.h"
#include "comms/field/IntValue.h"
#include "comms/options.h"
#include "ublox/DefaultOptions.h"
#include "ublox/MsgId.h"
#include "ublox/field/FieldBase.h"
#include "ublox/field/Lat.h"
#include "ublox/field/Lon.h"
#include "ublox/field/Res2.h"

namespace ublox
{

namespace message
{

/// @brief Fields of @ref MgaIniPosLlh.
/// @tparam TOpt Extra options
/// @see @ref MgaIniPosLlh
/// @headerfile "ublox/message/MgaIniPosLlh.h"
template <typename TOpt = ublox::DefaultOptions>
struct MgaIniPosLlhFields
{
    /// @brief Definition of <b>"type"</b> field.
    struct Type : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t,
            comms::option::FailOnInvalid<>,
            comms::option::DefaultNumValue<1>,
            comms::option::ValidNumValue<1>
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "type";
        }
        
    };
    
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
    
    /// @brief Definition of <b>"lat"</b> field.
    using Lat =
        ublox::field::Lat<
           TOpt
       >;
    
    /// @brief Definition of <b>"lon"</b> field.
    using Lon =
        ublox::field::Lon<
           TOpt
       >;
    
    /// @brief Definition of <b>"alt"</b> field.
    struct Alt : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::int32_t,
            comms::option::UnitsCentimeters
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "alt";
        }
        
    };
    
    /// @brief Definition of <b>"posAcc"</b> field.
    struct PosAcc : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint32_t,
            comms::option::UnitsCentimeters
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "posAcc";
        }
        
    };
    
    /// @brief All the fields bundled in std::tuple.
    using All = std::tuple<
        Type,
        Version,
        Reserved1,
        Lat,
        Lon,
        Alt,
        PosAcc
    >;
};

/// @brief Definition of <b>"MGA-BDS-POS_LLH"</b> message class.
/// @details
///     See @ref MgaIniPosLlhFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/MgaIniPosLlh.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class MgaIniPosLlh : public
    comms::MessageBase<
        TMsgBase,
        typename TOpt::message::MgaIniPosLlh,
        comms::option::StaticNumIdImpl<ublox::MsgId_MgaIni>,
        comms::option::FieldsImpl<typename MgaIniPosLlhFields<TOpt>::All>,
        comms::option::MsgType<MgaIniPosLlh<TMsgBase, TOpt> >,
        comms::option::HasName
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            typename TOpt::message::MgaIniPosLlh,
            comms::option::StaticNumIdImpl<ublox::MsgId_MgaIni>,
            comms::option::FieldsImpl<typename MgaIniPosLlhFields<TOpt>::All>,
            comms::option::MsgType<MgaIniPosLlh<TMsgBase, TOpt> >,
            comms::option::HasName
        >;

public:
    /// @brief Allow access to internal fields.
    /// @details See definition of @b COMMS_MSG_FIELDS_ACCESS macro
    ///     related to @b comms::MessageBase class from COMMS library
    ///     for details.
    ///
    ///     The generated functions are:
    ///     @li @b field_type() for @ref MgaIniPosLlhFields::Type field.
    ///     @li @b field_version() for @ref MgaIniPosLlhFields::Version field.
    ///     @li @b field_reserved1() for @ref MgaIniPosLlhFields::Reserved1 field.
    ///     @li @b field_lat() for @ref MgaIniPosLlhFields::Lat field.
    ///     @li @b field_lon() for @ref MgaIniPosLlhFields::Lon field.
    ///     @li @b field_alt() for @ref MgaIniPosLlhFields::Alt field.
    ///     @li @b field_posAcc() for @ref MgaIniPosLlhFields::PosAcc field.
    COMMS_MSG_FIELDS_ACCESS(
        type,
        version,
        reserved1,
        lat,
        lon,
        alt,
        posAcc
    );
    
    // Compile time check for serialisation length.
    static const std::size_t MsgMinLen = Base::doMinLength();
    static const std::size_t MsgMaxLen = Base::doMaxLength();
    static_assert(MsgMinLen == 20U, "Unexpected min serialisation length");
    static_assert(MsgMaxLen == 20U, "Unexpected max serialisation length");
    
    /// @brief Name of the message.
    static const char* doName()
    {
        return "MGA-BDS-POS_LLH";
    }
    
    
};

} // namespace message

} // namespace ublox


