/// @file
/// @brief Contains definition of <b>"CFG-ODO"</b> message and its fields.

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
#include "ublox/field/Res2.h"
#include "ublox/field/Res3.h"
#include "ublox/field/Res6.h"

namespace ublox
{

namespace message
{

/// @brief Fields of @ref CfgOdo.
/// @tparam TOpt Extra options
/// @see @ref CfgOdo
/// @headerfile "ublox/message/CfgOdo.h"
template <typename TOpt = ublox::DefaultOptions>
struct CfgOdoFields
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
    
    /// @brief Definition of <b>"reserved1"</b> field.
    struct Reserved1 : public
        ublox::field::Res3<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "reserved1";
        }
        
    };
    
    /// @brief Definition of <b>"flags"</b> field.
    class Flags : public
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
        ///      @li @b BitIdx_useODO, @b getBitValue_useODO() and @b setBitValue_useODO().
        ///      @li @b BitIdx_useCOG, @b getBitValue_useCOG() and @b setBitValue_useCOG().
        ///      @li @b BitIdx_outLPVel, @b getBitValue_outLPVel() and @b setBitValue_outLPVel().
        ///      @li @b BitIdx_outLPCog, @b getBitValue_outLPCog() and @b setBitValue_outLPCog().
        COMMS_BITMASK_BITS_SEQ(
            useODO,
            useCOG,
            outLPVel,
            outLPCog
        );
        
        /// @brief Name of the field.
        static const char* name()
        {
            return "flags";
        }
        
    };
    
    /// @brief Scope for all the member fields of @ref OdoCfg bitfield.
    struct OdoCfgMembers
    {
        /// @brief Values enumerator for @ref Profile field.
        enum class ProfileVal : std::uint8_t
        {
            Running = 0, ///< value @b Running
            Cycling = 1, ///< value @b Cycling
            Swimming = 2, ///< value @b Swimming
            Car = 3, ///< value @b Car
            Custom = 4, ///< value @b Custom
            
        };
        
        /// @brief Definition of <b>"profile"</b> field.
        struct Profile : public
            comms::field::EnumValue<
                ublox::field::FieldBase<>,
                ProfileVal,
                comms::option::FixedBitLength<3U>,
                comms::option::ValidNumValueRange<0, 4>
            >
        {
            /// @brief Name of the field.
            static const char* name()
            {
                return "profile";
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
               Profile,
               Reserved
            >;
    };
    
    /// @brief Definition of <b>"odoCfg"</b> field.
    class OdoCfg : public
        comms::field::Bitfield<
            ublox::field::FieldBase<>,
            typename OdoCfgMembers::All
        >
    {
        using Base = 
            comms::field::Bitfield<
                ublox::field::FieldBase<>,
                typename OdoCfgMembers::All
            >;
    public:
        /// @brief Allow access to internal fields.
        /// @details See definition of @b COMMS_FIELD_MEMBERS_ACCESS macro
        ///     related to @b comms::field::Bitfield class from COMMS library
        ///     for details.
        ///
        ///      The generated access functions are:
        ///     @li @b field_profile() - for OdoCfgMembers::Profile member field.
        ///     @li @b field_reserved() - for OdoCfgMembers::Reserved member field.
        COMMS_FIELD_MEMBERS_ACCESS(
            profile,
            reserved
        );
        
        /// @brief Name of the field.
        static const char* name()
        {
            return "odoCfg";
        }
        
    };
    
    /// @brief Definition of <b>"reserved2"</b> field.
    struct Reserved2 : public
        ublox::field::Res6<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "reserved2";
        }
        
    };
    
    /// @brief Definition of <b>"cogMaxSpeed"</b> field.
    struct CogMaxSpeed : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t,
            comms::option::ScalingRatio<1, 10>,
            comms::option::UnitsMetersPerSecond
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "cogMaxSpeed";
        }
        
    };
    
    /// @brief Definition of <b>"cogMaxPosAcc"</b> field.
    struct CogMaxPosAcc : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t,
            comms::option::UnitsMeters
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "cogMaxPosAcc";
        }
        
    };
    
    /// @brief Definition of <b>"reserved3"</b> field.
    struct Reserved3 : public
        ublox::field::Res2<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "reserved3";
        }
        
    };
    
    /// @brief Definition of <b>"velLpGain"</b> field.
    struct VelLpGain : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "velLpGain";
        }
        
    };
    
    /// @brief Definition of <b>"cogLpGain"</b> field.
    struct CogLpGain : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "cogLpGain";
        }
        
    };
    
    /// @brief Definition of <b>"reserved4"</b> field.
    struct Reserved4 : public
        ublox::field::Res2<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "reserved4";
        }
        
    };
    
    /// @brief All the fields bundled in std::tuple.
    using All = std::tuple<
        Version,
        Reserved1,
        Flags,
        OdoCfg,
        Reserved2,
        CogMaxSpeed,
        CogMaxPosAcc,
        Reserved3,
        VelLpGain,
        CogLpGain,
        Reserved4
    >;
};

/// @brief Definition of <b>"CFG-ODO"</b> message class.
/// @details
///     See @ref CfgOdoFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/CfgOdo.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class CfgOdo : public
    comms::MessageBase<
        TMsgBase,
        comms::option::StaticNumIdImpl<ublox::MsgId_CfgOdo>,
        comms::option::FieldsImpl<typename CfgOdoFields<TOpt>::All>,
        comms::option::MsgType<CfgOdo<TMsgBase, TOpt> >,
        comms::option::HasName
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            comms::option::StaticNumIdImpl<ublox::MsgId_CfgOdo>,
            comms::option::FieldsImpl<typename CfgOdoFields<TOpt>::All>,
            comms::option::MsgType<CfgOdo<TMsgBase, TOpt> >,
            comms::option::HasName
        >;

public:
    /// @brief Allow access to internal fields.
    /// @details See definition of @b COMMS_MSG_FIELDS_ACCESS macro
    ///     related to @b comms::MessageBase class from COMMS library
    ///     for details.
    ///
    ///     The generated functions are:
    ///     @li @b field_version() for @ref CfgOdoFields::Version field.
    ///     @li @b field_reserved1() for @ref CfgOdoFields::Reserved1 field.
    ///     @li @b field_flags() for @ref CfgOdoFields::Flags field.
    ///     @li @b field_odoCfg() for @ref CfgOdoFields::OdoCfg field.
    ///     @li @b field_reserved2() for @ref CfgOdoFields::Reserved2 field.
    ///     @li @b field_cogMaxSpeed() for @ref CfgOdoFields::CogMaxSpeed field.
    ///     @li @b field_cogMaxPosAcc() for @ref CfgOdoFields::CogMaxPosAcc field.
    ///     @li @b field_reserved3() for @ref CfgOdoFields::Reserved3 field.
    ///     @li @b field_velLpGain() for @ref CfgOdoFields::VelLpGain field.
    ///     @li @b field_cogLpGain() for @ref CfgOdoFields::CogLpGain field.
    ///     @li @b field_reserved4() for @ref CfgOdoFields::Reserved4 field.
    COMMS_MSG_FIELDS_ACCESS(
        version,
        reserved1,
        flags,
        odoCfg,
        reserved2,
        cogMaxSpeed,
        cogMaxPosAcc,
        reserved3,
        velLpGain,
        cogLpGain,
        reserved4
    );
    
    // Compile time check for serialisation length.
    static const std::size_t MsgMinLen = Base::doMinLength();
    static const std::size_t MsgMaxLen = Base::doMaxLength();
    static_assert(MsgMinLen == 20U, "Unexpected min serialisation length");
    static_assert(MsgMaxLen == 20U, "Unexpected max serialisation length");
    
    /// @brief Name of the message.
    static const char* doName()
    {
        return "CFG-ODO";
    }
    
    
};

} // namespace message

} // namespace ublox


