/// @file
/// @brief Contains definition of <b>"CFG-GEOFENCE"</b> message and its fields.

#pragma once

#include <cstdint>
#include <tuple>
#include "comms/MessageBase.h"
#include "comms/field/ArrayList.h"
#include "comms/field/Bundle.h"
#include "comms/field/EnumValue.h"
#include "comms/field/IntValue.h"
#include "comms/options.h"
#include "ublox/DefaultOptions.h"
#include "ublox/MsgId.h"
#include "ublox/field/FieldBase.h"
#include "ublox/field/Lat.h"
#include "ublox/field/Lon.h"
#include "ublox/field/Res1.h"

namespace ublox
{

namespace message
{

/// @brief Fields of @ref CfgGeofence.
/// @tparam TOpt Extra options
/// @see @ref CfgGeofence
/// @headerfile "ublox/message/CfgGeofence.h"
template <typename TOpt = ublox::DefaultOptions>
struct CfgGeofenceFields
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
    
    /// @brief Definition of <b>"numFences"</b> field.
    struct NumFences : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "numFences";
        }
        
    };
    
    /// @brief Definition of <b>"confLvl"</b> field.
    struct ConfLvl : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "confLvl";
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
    
    /// @brief Values enumerator for @ref PioEnabled field.
    enum class PioEnabledVal : std::uint8_t
    {
        Disable = 0, ///< value @b Disable
        Enable = 1, ///< value @b Enable
        
    };
    
    /// @brief Definition of <b>"pioEnabled"</b> field.
    struct PioEnabled : public
        comms::field::EnumValue<
            ublox::field::FieldBase<>,
            PioEnabledVal,
            comms::option::ValidNumValueRange<0, 1>
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "pioEnabled";
        }
        
    };
    
    /// @brief Values enumerator for @ref PinPolarity field.
    enum class PinPolarityVal : std::uint8_t
    {
        LowInside = 0, ///< value @b LowInside
        LowOutside = 1, ///< value @b LowOutside
        
    };
    
    /// @brief Definition of <b>"pinPolarity"</b> field.
    struct PinPolarity : public
        comms::field::EnumValue<
            ublox::field::FieldBase<>,
            PinPolarityVal,
            comms::option::ValidNumValueRange<0, 1>
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "pinPolarity";
        }
        
    };
    
    /// @brief Definition of <b>"pin"</b> field.
    struct Pin : public
        comms::field::IntValue<
            ublox::field::FieldBase<>,
            std::uint8_t
        >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "pin";
        }
        
    };
    
    /// @brief Definition of <b>"reserved2"</b> field.
    struct Reserved2 : public
        ublox::field::Res1<
           TOpt
       >
    {
        /// @brief Name of the field.
        static const char* name()
        {
            return "reserved2";
        }
        
    };
    
    /// @brief Scope for all the member fields of @ref List list.
    struct ListMembers
    {
        /// @brief Scope for all the member fields of @ref Element bitfield.
        struct ElementMembers
        {
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
            
            /// @brief Definition of <b>"radius"</b> field.
            struct Radius : public
                comms::field::IntValue<
                    ublox::field::FieldBase<>,
                    std::uint32_t,
                    comms::option::ScalingRatio<100, 1>,
                    comms::option::UnitsMeters
                >
            {
                /// @brief Name of the field.
                static const char* name()
                {
                    return "radius";
                }
                
            };
            
            /// @brief All members bundled in @b std::tuple.
            using All =
                std::tuple<
                   Lat,
                   Lon,
                   Radius
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
            ///     @li @b field_lat() - for ElementMembers::Lat member field.
            ///     @li @b field_lon() - for ElementMembers::Lon member field.
            ///     @li @b field_radius() - for ElementMembers::Radius member field.
            COMMS_FIELD_MEMBERS_ACCESS(
                lat,
                lon,
                radius
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
            typename TOpt::message::CfgGeofenceFields::List,
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
        Version,
        NumFences,
        ConfLvl,
        Reserved1,
        PioEnabled,
        PinPolarity,
        Pin,
        Reserved2,
        List
    >;
};

/// @brief Definition of <b>"CFG-GEOFENCE"</b> message class.
/// @details
///     See @ref CfgGeofenceFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/CfgGeofence.h"
template <typename TMsgBase, typename TOpt = ublox::DefaultOptions>
class CfgGeofence : public
    comms::MessageBase<
        TMsgBase,
        comms::option::StaticNumIdImpl<ublox::MsgId_CfgGeofence>,
        comms::option::FieldsImpl<typename CfgGeofenceFields<TOpt>::All>,
        comms::option::MsgType<CfgGeofence<TMsgBase, TOpt> >,
        comms::option::HasName,
        comms::option::HasCustomRefresh
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            comms::option::StaticNumIdImpl<ublox::MsgId_CfgGeofence>,
            comms::option::FieldsImpl<typename CfgGeofenceFields<TOpt>::All>,
            comms::option::MsgType<CfgGeofence<TMsgBase, TOpt> >,
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
    ///     @li @b field_version() for @ref CfgGeofenceFields::Version field.
    ///     @li @b field_numFences() for @ref CfgGeofenceFields::NumFences field.
    ///     @li @b field_confLvl() for @ref CfgGeofenceFields::ConfLvl field.
    ///     @li @b field_reserved1() for @ref CfgGeofenceFields::Reserved1 field.
    ///     @li @b field_pioEnabled() for @ref CfgGeofenceFields::PioEnabled field.
    ///     @li @b field_pinPolarity() for @ref CfgGeofenceFields::PinPolarity field.
    ///     @li @b field_pin() for @ref CfgGeofenceFields::Pin field.
    ///     @li @b field_reserved2() for @ref CfgGeofenceFields::Reserved2 field.
    ///     @li @b field_list() for @ref CfgGeofenceFields::List field.
    COMMS_MSG_FIELDS_ACCESS(
        version,
        numFences,
        confLvl,
        reserved1,
        pioEnabled,
        pinPolarity,
        pin,
        reserved2,
        list
    );
    
    // Compile time check for serialisation length.
    static const std::size_t MsgMinLen = Base::doMinLength();
    static_assert(MsgMinLen == 8U, "Unexpected min serialisation length");
    
    /// @brief Name of the message.
    static const char* doName()
    {
        return "CFG-GEOFENCE";
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
            static_cast<std::size_t>(field_numFences().value()));
        
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
            auto expectedValue = static_cast<std::size_t>(field_numFences().value());
            auto realValue = field_list().value().size();
            if (expectedValue != realValue) {
                using PrefixValueType = typename std::decay<decltype(field_numFences().value())>::type;
                field_numFences().value() = static_cast<PrefixValueType>(realValue);
                updated = true;
            }
        } while (false);
        
        return updated;
        
    }
    
};

} // namespace message

} // namespace ublox


