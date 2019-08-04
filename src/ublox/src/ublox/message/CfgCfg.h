/// @file
/// @brief Contains definition of <b>"CFG-CFG"</b> message and its fields.

#pragma once

#include <tuple>
#include "comms/MessageBase.h"
#include "comms/field/BitmaskValue.h"
#include "comms/field/Optional.h"
#include "comms/options.h"
#include "ublox/DefaultOptions.h"
#include "ublox/MsgId.h"
#include "ublox/field/CfgCfgMask.h"
#include "ublox/field/FieldBase.h"

namespace ublox
{

namespace message
{

/// @brief Fields of @ref CfgCfg.
/// @tparam TOpt Extra options
/// @see @ref CfgCfg
/// @headerfile "ublox/message/CfgCfg.h"
template <typename TOpt = ublox::DefaultOptions>
struct CfgCfgFields
{
    /// @brief Definition of <b>"clearMask"</b> field.
    struct ClearMask : public
        ublox::field::CfgCfgMask<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "clearMask";
        }
        
    };
    
    /// @brief Definition of <b>"saveMask"</b> field.
    struct SaveMask : public
        ublox::field::CfgCfgMask<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "saveMask";
        }
        
    };
    
    /// @brief Definition of <b>"loadMask"</b> field.
    struct LoadMask : public
        ublox::field::CfgCfgMask<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "loadMask";
        }
        
    };
    
    /// @brief Scope for all the member fields of @ref DeviceMask optional.
    struct DeviceMaskMembers
    {
        /// @brief Definition of <b>"deviceMask"</b> field.
        class DeviceMask : public
            comms::field::BitmaskValue<
                ublox::field::FieldBase<>,
                comms::option::FixedLength<1U>,
                comms::option::BitmaskReservedBits<0xE8U, 0x0U>
            >
        {
            using Base = 
                comms::field::BitmaskValue<
                    ublox::field::FieldBase<>,
                    comms::option::FixedLength<1U>,
                    comms::option::BitmaskReservedBits<0xE8U, 0x0U>
                >;
        public:
            /// @brief Provide names for internal bits.
            /// @details See definition of @b COMMS_BITMASK_BITS macro
            ///     related to @b comms::field::BitmaskValue class from COMMS library
            ///     for details.
            ///
            ///      The generated enum values:
            ///      @li @b BitIdx_devBBR.
            ///      @li @b BitIdx_devFlash.
            ///      @li @b BitIdx_devEEPROM.
            ///      @li @b BitIdx_devSpiFlash.
            COMMS_BITMASK_BITS(
                devBBR=0,
                devFlash=1,
                devEEPROM=2,
                devSpiFlash=4
            );
            
            /// @brief Generates independent access functions for internal bits.
            /// @details See definition of @b COMMS_BITMASK_BITS_ACCESS macro
            ///     related to @b comms::field::BitmaskValue class from COMMS library
            ///     for details.
            ///
            ///     The generated access functions are:
            ///      @li @b getBitValue_devBBR() and @b setBitValue_devBBR().
            ///      @li @b getBitValue_devFlash() and @b setBitValue_devFlash().
            ///      @li @b getBitValue_devEEPROM() and @b setBitValue_devEEPROM().
            ///      @li @b getBitValue_devSpiFlash() and @b setBitValue_devSpiFlash().
            COMMS_BITMASK_BITS_ACCESS(
                devBBR,
                devFlash,
                devEEPROM,
                devSpiFlash
            );
            
            /// @brief Name of the field.
            static const char* name()
            {
                return "deviceMask";
            }
            
        };
        
    };
    
    /// @brief Definition of <b>"deviceMask"</b> field.
    struct DeviceMask : public
        comms::field::Optional<
            typename DeviceMaskMembers::DeviceMask
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "deviceMask";
        }
        
    };
    
    /// @brief All the fields bundled in std::tuple.
    using All = std::tuple<
        ClearMask,
        SaveMask,
        LoadMask,
        DeviceMask
    >;
};

/// @brief Definition of <b>"CFG-CFG"</b> message class.
/// @details
///     See @ref CfgCfgFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/CfgCfg.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class CfgCfg : public
    comms::MessageBase<
        TMsgBase,
        typename TOpt::message::CfgCfg,
        comms::option::StaticNumIdImpl<ublox::MsgId_CfgCfg>,
        comms::option::FieldsImpl<typename CfgCfgFields<TOpt>::All>,
        comms::option::MsgType<CfgCfg<TMsgBase, TOpt> >,
        comms::option::HasName
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            typename TOpt::message::CfgCfg,
            comms::option::StaticNumIdImpl<ublox::MsgId_CfgCfg>,
            comms::option::FieldsImpl<typename CfgCfgFields<TOpt>::All>,
            comms::option::MsgType<CfgCfg<TMsgBase, TOpt> >,
            comms::option::HasName
        >;

public:
    /// @brief Allow access to internal fields.
    /// @details See definition of @b COMMS_MSG_FIELDS_ACCESS macro
    ///     related to @b comms::MessageBase class from COMMS library
    ///     for details.
    ///
    ///     The generated functions are:
    ///     @li @b field_clearMask() for @ref CfgCfgFields::ClearMask field.
    ///     @li @b field_saveMask() for @ref CfgCfgFields::SaveMask field.
    ///     @li @b field_loadMask() for @ref CfgCfgFields::LoadMask field.
    ///     @li @b field_deviceMask() for @ref CfgCfgFields::DeviceMask field.
    COMMS_MSG_FIELDS_ACCESS(
        clearMask,
        saveMask,
        loadMask,
        deviceMask
    );
    
    // Compile time check for serialisation length.
    static const std::size_t MsgMinLen = Base::doMinLength();
    static const std::size_t MsgMaxLen = Base::doMaxLength();
    static_assert(MsgMinLen == 12U, "Unexpected min serialisation length");
    static_assert(MsgMaxLen == 13U, "Unexpected max serialisation length");
    
    /// @brief Name of the message.
    static const char* doName()
    {
        return "CFG-CFG";
    }
    
    
};

} // namespace message

} // namespace ublox


