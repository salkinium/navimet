//
// Copyright 2017 (C). Alex Robenko. All rights reserved.
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


#pragma once

#include <type_traits>
#include <memory>

#include "comms/Assert.h"
#include "comms/util/Tuple.h"
#include "comms/util/alloc.h"
#include "comms/MessageBase.h"

namespace comms
{

namespace details
{

template <typename TMessage, bool TIsMessageBase>
struct MsgFactoryMessageHasStaticNumId;

template <typename TMessage>
struct MsgFactoryMessageHasStaticNumId<TMessage, true>
{
    static const bool Value = TMessage::ImplOptions::HasStaticMsgId;
};

template <typename TMessage>
struct MsgFactoryMessageHasStaticNumId<TMessage, false>
{
    static const bool Value = false;
};

struct MsgFactoryStaticNumIdCheckHelper
{
    template <typename TMessage>
    constexpr bool operator()(bool value) const
    {
        return value && MsgFactoryMessageHasStaticNumId<TMessage, comms::isMessageBase<TMessage>()>::Value;
    }
};

template <typename TAllMessages>
constexpr bool msgFactoryAllHaveStaticNumId()
{
    return comms::util::tupleTypeAccumulate<TAllMessages>(true, MsgFactoryStaticNumIdCheckHelper());
}

template <typename TMessage>
constexpr bool msgFactoryMessageHasStaticNumId()
{
    return MsgFactoryMessageHasStaticNumId<TMessage, comms::isMessageBase<TMessage>()>::Value;
}

template<bool TMustCat>
struct AllMessagesRetrieveHelper;

template<>
struct AllMessagesRetrieveHelper<true>
{
    template <typename TAll, typename TOpt>
    using Type =
        typename std::decay<
            decltype(
                std::tuple_cat(
                    std::declval<TAll>(),
                    std::declval<std::tuple<typename TOpt::GenericMessage> >()
                )
            )
        >::type;
};

template<>
struct AllMessagesRetrieveHelper<false>
{
    template <typename TAll, typename TOpt>
    using Type = TAll;
};

template <typename TAll, typename TOpt>
using AllMessagesBundle =
    typename AllMessagesRetrieveHelper<TOpt::HasInPlaceAllocation && TOpt::HasSupportGenericMessage>::template Type<TAll, TOpt>;

template <typename TMsgBase, typename TAllMessages, typename... TOptions>
class MsgFactoryBase
{
    static_assert(TMsgBase::InterfaceOptions::HasMsgIdType,
        "Usage of MsgFactoryBase requires Message interface to provide ID type. "
        "Use comms::option::MsgIdType option in message interface type definition.");
    using ParsedOptionsInternal = details::MsgFactoryOptionsParser<TOptions...>;

    using AllMessagesInternal = AllMessagesBundle<TAllMessages, ParsedOptionsInternal>;
    using Alloc =
        typename std::conditional<
            ParsedOptionsInternal::HasInPlaceAllocation,
            util::alloc::InPlaceSingle<TMsgBase, AllMessagesInternal>,
            util::alloc::DynMemory<TMsgBase>
        >::type;
public:
    using ParsedOptions = ParsedOptionsInternal;
    using Message = TMsgBase;
    using MsgIdParamType = typename Message::MsgIdParamType;
    using MsgIdType = typename Message::MsgIdType;
    using MsgPtr = typename Alloc::Ptr;
    using AllMessages = TAllMessages;

    MsgPtr createGenericMsg(MsgIdParamType id) const
    {
        static_cast<void>(this);
        using Tag =
            typename std::conditional<
                ParsedOptions::HasSupportGenericMessage,
                AllocGenericTag,
                NoAllocTag
            >::type;

        return createGenericMsgInternal(id, Tag());
    }


protected:
    MsgFactoryBase() = default;
    MsgFactoryBase(const MsgFactoryBase&) = default;
    MsgFactoryBase(MsgFactoryBase&&) = default;
    MsgFactoryBase& operator=(const MsgFactoryBase&) = default;
    MsgFactoryBase& operator=(MsgFactoryBase&&) = default;

    class FactoryMethod
    {
    public:
        MsgIdParamType getId() const
        {
            return getIdImpl();
        }

        MsgPtr create(const MsgFactoryBase& factory) const
        {
            return createImpl(factory);
        }

    protected:
        FactoryMethod() = default;

        virtual MsgIdParamType getIdImpl() const = 0;
        virtual MsgPtr createImpl(const MsgFactoryBase& factory) const = 0;
    };

    template <typename TMessage>
    class NumIdFactoryMethod : public FactoryMethod
    {
    public:
        using Message = TMessage;
        static const decltype(Message::MsgId) MsgId = Message::MsgId;
        NumIdFactoryMethod() {}

    protected:
        virtual MsgIdParamType getIdImpl() const
        {
            return static_cast<MsgIdParamType>(MsgId);
        }

        virtual MsgPtr createImpl(const MsgFactoryBase& factory) const
        {
            return factory.template allocMsg<Message>();
        }
    };

    template <typename TMessage>
    friend class NumIdFactoryMethod;

    template <typename TMessage>
    class GenericFactoryMethod : public FactoryMethod
    {
    public:
        using Message = TMessage;

        GenericFactoryMethod() : id_(Message().getId()) {}

    protected:

        virtual MsgIdParamType getIdImpl() const
        {
            return id_;
        }

        virtual MsgPtr createImpl(const MsgFactoryBase& factory) const
        {
            return factory.template allocMsg<Message>();
        }

    private:
        typename Message::MsgIdType id_;
    };

    template <typename TMessage>
    friend class GenericFactoryMethod;

    template <typename TObj, typename... TArgs>
    MsgPtr allocMsg(TArgs&&... args) const
    {
        static_assert(std::is_base_of<Message, TObj>::value,
            "TObj is not a proper message type");

        static_assert(
            (!ParsedOptionsInternal::HasInPlaceAllocation) ||
                    comms::util::IsInTuple<TObj, AllMessagesInternal>::Value,
            "TObj must be in provided tuple of supported messages");

        return alloc_.template alloc<TObj>(std::forward<TArgs>(args)...);
    }

private:
    struct AllocGenericTag {};
    struct NoAllocTag {};

    MsgPtr createGenericMsgInternal(MsgIdParamType id, AllocGenericTag) const
    {
        static_assert(std::is_base_of<Message, typename ParsedOptions::GenericMessage>::value,
            "The requested GenericMessage class must have the same interface class as all other messages");
        return allocMsg<typename ParsedOptions::GenericMessage>(id);
    }

    static MsgPtr createGenericMsgInternal(MsgIdParamType, NoAllocTag)
    {
        return MsgPtr();
    }

    mutable Alloc alloc_;
};


}  // namespace details

}  // namespace comms


