//
// Copyright 2015 - 2017 (C). Alex Robenko. All rights reserved.
//

// This file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/// @file
/// Contains definition of comms::MsgFactory class.

#pragma once

#include <type_traits>
#include <algorithm>

#include "comms/Assert.h"
#include "comms/util/Tuple.h"
#include "comms/util/alloc.h"
#include "details/MsgFactoryOptionsParser.h"
#include "details/MsgFactorySelector.h"

namespace comms
{

/// @brief Message factory class.
/// @details It is responsible to create message objects given the ID of the
///     message. This class @b DOESN'T use dynamic memory allocation to store its
///     internal data structures, hence can be used in any bare-metal and other
///     embedded environment.@n
///     The types of all messages provided in @b TAllMessages are analysed at
///     compile time and best "id to message object" mapping strategy is chosen,
///     whether it is direct access array (with O(1) time complexity) or
///     sorted array with binary search (with O(log(n)) time complexity).
/// @tparam TMsgBase Common base class for all the messages, smart pointer to
///     this type is returned when allocation of specify message is requested.
/// @tparam TAllMessages All custom message types, that this factory is capable
///     of creating, bundled in std::tuple<>. The message types must be sorted
///     based on their IDs. Different variants of the same message (reporting
///     same ID, but implemented as different classes) are also supported. However
///     they must follow one another in this std::tuple, i.e. be sorted.
/// @tparam TOptions Zero or more options. The supported options are:
///     @li comms::option::InPlaceAllocation - Option to specify that custom
///         message objects are @b NOT allocated using dynamic memory, instead
///         an uninitialised area of memory in private members is used to contain
///         any type of custom message (provided with TAllMessages template parameter) and
///         placement "new" operator is used to initialise requested message in
///         this area.
///         The allocated message objects are returned from createMsg() function
///         wrapped in the smart pointer (variant of std::unique_ptr). If
///         comms::option::InPlaceAllocation option is used, then the smart pointer
///         definition contains custom deleter, which will explicitly invoke
///         destructor of the message when the smart pointer is out of scope. It
///         means that it is @b NOT possible to create new message with this factory
///         if previously allocated one wasn't destructed yet.
///         If comms::option::InPlaceAllocation option is NOT used, than the
///         requested message objects are allocated using dynamic memory and
///         returned wrapped in std::unique_ptr without custom deleter.
///     @li comms::option::SupportGenericMessage - Option used to allow
///         allocation of @ref comms::GenericMessage. If such option is
///         provided, the createGenericMsg() member function will be able
///         to allocate @ref comms::GenericMessage object. @b NOTE, that
///         the base class of @ref comms::GenericMessage type (first template
///         parameter) must be equal to @b TMsgBase (first template parameter)
///         of @b this class.
/// @pre TMsgBase is a base class for all the messages in TAllMessages.
/// @pre Message type is TAllMessages must be sorted based on their IDs.
/// @pre If comms::option::InPlaceAllocation option is provided, only one custom
///     message can be allocated. The next one can be allocated only after previous
///     message has been destructed.
/// @headerfile comms/MsgFactory.h
template <typename TMsgBase, typename TAllMessages, typename... TOptions>
class MsgFactory
{
    static_assert(TMsgBase::InterfaceOptions::HasMsgIdType,
        "Usage of MsgFactory requires Message interface to provide ID type. "
        "Use comms::option::MsgIdType option in message interface type definition.");

    using Factory = typename
        details::MsgFactorySelector<TMsgBase, TAllMessages, TOptions...>::Type;

public:
    /// @brief Parsed options
    using ParsedOptions = typename Factory::ParsedOptions;

    /// @brief Type of the common base class of all the messages.
    using Message = TMsgBase;

    /// @brief Type of the message ID when passed as a parameter.
    using MsgIdParamType = typename Message::MsgIdParamType;

    /// @brief Type of the message ID.
    using MsgIdType = typename Message::MsgIdType;

    /// @brief Smart pointer to @ref Message which holds allocated message object.
    /// @details It is a variant of std::unique_ptr, based on whether
    ///     comms::option::InPlaceAllocation option was used.
    using MsgPtr = typename Factory::MsgPtr;

    /// @brief All messages provided as template parameter to this class.
    using AllMessages = TAllMessages;

    /// @brief Create message object given the ID of the message.
    /// @param id ID of the message.
    /// @param idx Relative index of the message with the same ID. In case
    ///     protocol implementation contains multiple distinct message types
    ///     that report same ID value, it must be possible to choose the
    ///     relative index of such message from the first message type reporting
    ///     the same ID. This parameter provides such an ability. However,
    ///     most protocols will implement single message class for single ID.
    ///     For such implementations, use default value of this parameter.
    /// @return Smart pointer (variant of std::unique_ptr) to @ref Message type,
    ///     which is a common base class of all the messages (provided as
    ///     first template parameter to this class). If comms::option::InPlaceAllocation
    ///     option was used and previously allocated message wasn't de-allocated
    ///     yet, the empty (null) pointer will be returned.
    MsgPtr createMsg(MsgIdParamType id, unsigned idx = 0) const
    {
        return factory_.createMsg(id, idx);
    }

    /// @brief Allocate and initialise @ref comms::GenericMessage object.
    /// @details If comms::option::SupportGenericMessage option hasn't been
    ///     provided, this function will return empty @b MsgPtr pointer. Otherwise
    ///     the relevant allocator will be used to allocate @ref comms::GenericMessage.
    /// @param[in] id ID of the message, will be passed as a parameter to the
    ///     constructor of the @ref comms::GenericMessage class
    MsgPtr createGenericMsg(MsgIdParamType id) const
    {
        return factory_.createGenericMsg(id);
    }

    /// @brief Get number of message types from @ref AllMessages, that have the specified ID.
    /// @param id ID of the message.
    /// @return Number of message classes that report same ID.
    std::size_t msgCount(MsgIdParamType id) const
    {
        return factory_.msgCount(id);
    }

    /// @brief Compile time knowldege inquiry whether all the message classes in the
    ///     @b TAllMessages bundle have unique IDs.
    static constexpr bool hasUniqueIds()
    {
        return Factory::hasUniqueIds();
    }

private:
    Factory factory_;
};


}  // namespace comms

