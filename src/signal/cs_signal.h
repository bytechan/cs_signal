/***********************************************************************
*
* Copyright (c) 2016-2025 Barbara Geller
* Copyright (c) 2016-2025 Ansel Sermersheim
*
* This file is part of CsSignal.
*
* CsSignal is free software which is released under the BSD 2-Clause license.
* For license details refer to the LICENSE provided with this project.
*
* CsSignal is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*
* https://opensource.org/licenses/BSD-2-Clause
*
***********************************************************************/

#ifndef LIB_CS_SIGNAL_H
#define LIB_CS_SIGNAL_H

#include <algorithm>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <tuple>
#include <unordered_set>

#include "cs_internal.h"
#include "cs_macro.h"
#include "cs_slot.h"
#include "cs_rcu_guarded.h"
#include "cs_rcu_list.h"

namespace CsSignal {

enum class ConnectionKind {
   AutoConnection,
   DirectConnection,
   QueuedConnection,
   BlockingQueuedConnection
};

enum class DisconnectKind {
   DisconnectAll,
   DisconnectOne
};

template <class Iter1, class Iter2, class T>
Iter1 find(Iter1 iter1, const Iter2 &iter2, const T &value)
{
   while (iter1 != iter2) {
      if (value == *iter1) {
         break;
      }

      ++iter1;
   }

   return iter1;
}

template<class Sender, class SignalClass, class ...SignalArgs, class Receiver,
                  class SlotClass, class ...SlotArgs, class SlotReturn>
bool connect(const Sender &sender, void (SignalClass::*signalMethod)(SignalArgs...),
                  const Receiver &receiver, SlotReturn (SlotClass::*slotMethod)(SlotArgs...),
                  ConnectionKind type = ConnectionKind::AutoConnection, bool uniqueConnection = false);

template<class Sender, class SignalClass, class ...SignalArgs, class Receiver, class T>
bool connect(const Sender &sender, void (SignalClass::*signalMethod)(SignalArgs...),
                  const Receiver &receiver, T slotLambda,
                  ConnectionKind type = ConnectionKind::AutoConnection, bool uniqueConnection = false);

template<class Sender, class Receiver>
bool connect(const Sender &sender, std::unique_ptr<Internal::BentoAbstract> signalMethod_Bento,
                  const Receiver &receiver, std::unique_ptr<Internal::BentoAbstract> slotMethod_Bento,
                  ConnectionKind type = ConnectionKind::AutoConnection, bool uniqueConnection = false);

// base class
class LIB_SIG_EXPORT SignalBase
{
   public:
      virtual ~SignalBase();

   protected:
      static Internal::BentoAbstract *&get_threadLocal_currentSignal();

      int internal_cntConnections(const SlotBase *receiver,
                  const Internal::BentoAbstract &signalMethod_Bento) const;

      std::set<SlotBase *> internal_receiverList(
                  const Internal::BentoAbstract &signalMethod_Bento) const;

   private:
      // part of destructor
      static std::mutex &get_mutex_beingDestroyed();
      static std::unordered_set<const SignalBase *> &get_beingDestroyed();

      // part of disconnect
      mutable int m_activateBusy = 0;

      struct ConnectStruct {
         std::unique_ptr<const Internal::BentoAbstract> signalMethod;
         const SlotBase *receiver;
         std::unique_ptr<const Internal::BentoAbstract> slotMethod;
         ConnectionKind type;
      };

      // list of connections from my Signal to some Receiver
      mutable libguarded::SharedList<ConnectStruct> m_connectList;

      void addConnection(std::unique_ptr<const Internal::BentoAbstract> signalMethod, const SlotBase *,
                  std::unique_ptr<const Internal::BentoAbstract> slotMethod, ConnectionKind type,
                  libguarded::SharedList<ConnectStruct>::write_handle &senderListHandle) const;

      virtual void handleException(std::exception_ptr data);

      template<class Sender, class SignalClass, class ...SignalArgTypes, class ...Ts>
      friend void activate(Sender &sender, void (SignalClass::*signal)(SignalArgTypes...), Ts &&... Vs);

      template<class Sender, class SignalClass, class ...SignalArgs, class Receiver,
                  class SlotClass, class ...SlotArgs, class SlotReturn>
      friend bool connect(const Sender &sender, void (SignalClass::*signalMethod)(SignalArgs...),
                  const Receiver &receiver, SlotReturn (SlotClass::*slotMethod)(SlotArgs...),
                  ConnectionKind type, bool uniqueConnection);

      template<class Sender, class SignalClass, class ...SignalArgs, class Receiver, class T>
      friend bool connect(const Sender &sender, void (SignalClass::*signalMethod)(SignalArgs...),
                  const Receiver &receiver, T slotLambda,
                  ConnectionKind type, bool uniqueConnection);

      template<class Sender, class Receiver>
      friend bool connect(const Sender &sender, std::unique_ptr<Internal::BentoAbstract> signalMethod_Bento,
                  const Receiver &receiver, std::unique_ptr<Internal::BentoAbstract> slotMethod_Bento,
                  ConnectionKind type, bool uniqueConnection);

      template<class Sender, class Receiver>
      friend bool internal_disconnect(const Sender &sender, const Internal::BentoAbstract *signalBento,
                  const Receiver *receiver, const Internal::BentoAbstract *slotBento);

      friend class SlotBase;
};

template<class Sender, class SignalClass, class ...SignalArgTypes, class ...Ts>
void activate(Sender &sender, void (SignalClass::*signal)(SignalArgTypes...), Ts &&... Vs)
{
   // ensure signal args are passed
   static_assert( std::is_convertible<std::tuple<Ts...>, std::tuple<SignalArgTypes...>>::value,
                  "activate():  Signal parameter mismatch.");

   Internal::Bento<void (SignalClass::*)(SignalArgTypes...)> signal_Bento(signal);

   // save the address of sender
   const SignalBase *senderPtr = &sender;

   // store the signal data, false indicates the data will not be copied
   Internal::TeaCup_Data<SignalArgTypes...> dataPack(false, std::forward<Ts>(Vs)...);

   SignalBase *priorSender = SlotBase::get_threadLocal_currentSender();
   SlotBase::get_threadLocal_currentSender() = &sender;

   Internal::BentoAbstract *priorSignal = SignalBase::get_threadLocal_currentSignal();
   SignalBase::get_threadLocal_currentSignal() = &signal_Bento;

   // threading and queuedConnections
   auto senderListHandle = sender.m_connectList.lock_read();
   auto iter = senderListHandle->begin();
   const auto end = senderListHandle->end();

   while (iter != end) {
      const auto &connection = *iter;

      if (*(connection.signalMethod) != signal_Bento) {
         // no match in connectionList for this signal
         ++iter;
         continue;
      }

      SlotBase *receiver = const_cast<SlotBase *>(connection.receiver);

      // const reference to a unique ptr
      const std::unique_ptr<const CsSignal::Internal::BentoAbstract> &slot_Bento = connection.slotMethod;

      ++iter;

      if (! receiver) {
         continue;
      }

      // check if this is a queued connection
      if (connection.type == ConnectionKind::QueuedConnection ||
            (connection.type == ConnectionKind::AutoConnection && ! receiver->compareThreads())) {

         // create a copy of the signal parameters
         auto teaCup_Data = Internal::make_unique<Internal::TeaCup_Data<SignalArgTypes...>>(true, std::forward<Ts>(Vs)...);

         // make a copy of the signal and slot bentos
         std::unique_ptr<Internal::BentoAbstract> signal_Bento_Copy = signal_Bento.clone();
         std::unique_ptr<Internal::BentoAbstract> slot_Bento_Copy   = slot_Bento->clone();

         // create a PendingSlot, this will be added to the receivers event queue
         PendingSlot data(&sender, std::move(signal_Bento_Copy), receiver,
                  std::move(slot_Bento_Copy), std::move(teaCup_Data));

         receiver->queueSlot(std::move(data), connection.type);

         continue;
      }

      try {
         // direct connection
         Internal::TeaCup_Data<SignalArgTypes...> dataPack(false, std::forward<Ts>(Vs)...);
         slot_Bento->invoke(receiver, &dataPack);

      } catch (...) {
         // catch and save the exception
         sender.handleException(std::current_exception());
      }
   }

   SignalBase::get_threadLocal_currentSignal() = priorSignal;
   SlotBase::get_threadLocal_currentSender()   = priorSender;
}

// signal & slot method ptr
template<class Sender, class SignalClass, class ...SignalArgs, class Receiver, class SlotClass,
                  class ...SlotArgs, class SlotReturn>
bool connect(const Sender &sender, void (SignalClass::*signalMethod)(SignalArgs...),
                  const Receiver &receiver, SlotReturn (SlotClass::*slotMethod)(SlotArgs...),
                  ConnectionKind type, bool uniqueConnection)
{

/*
   // is the sender an rvalue reference
   static_assert(! std::is_rvalue_reference<Sender &&>::value,
                  "connect():  Sender can not be an rvalue");

   // is the receiver an rvalue reference
   static_assert(! std::is_rvalue_reference<Receiver &&>::value,
                  "connect():  Receiver can not be an rvalue");
*/

   // (1) Sender must be the same class as SignalClass OR (2) Sender is a child of SignalClass
   static_assert( std::is_base_of<SignalClass, Sender>::value,
                  "connect():  Signal was not a child class of Sender");

   // (1) Receiver must be the same class as SlotClass OR (2) Receiver is a child of SlotClass
   static_assert( std::is_base_of<SlotClass, Receiver>::value,
                  "connect():  Slot was not a child class of Receiver");

   // compare signal and slot parameter list
   static_assert( Internal::cs_check_connect_args<void (*)(SignalArgs...), void (*)(SlotArgs...) >::value,
                  "connect():  Incompatible signal/slot arguments");

   if (signalMethod == nullptr) {
      throw std::invalid_argument("connect() Can not connect, signal is null");
   }

   if (slotMethod == nullptr) {
      throw std::invalid_argument("connect(): Can not connect, slot is null");
   }

   std::unique_ptr<Internal::Bento<void (SignalClass::*)(SignalArgs...)>>
                  signalMethod_Bento(new Internal::Bento<void (SignalClass::*)(SignalArgs...)>(signalMethod));

   std::unique_ptr<Internal::Bento<SlotReturn (SlotClass::*)(SlotArgs...)>>
                  slotMethod_Bento(new Internal::Bento<SlotReturn (SlotClass::*)(SlotArgs...)>(slotMethod));

   auto senderListHandle = sender.m_connectList.lock_write();

   if (uniqueConnection) {
      // ensure the connection is not added twice
      auto iter = senderListHandle->begin();
      const auto end = senderListHandle->end();

      while (iter != end) {
         const auto &item = *iter;

         if (item.receiver != &receiver) {
            ++iter;
            continue;
         }

         if (*(item.signalMethod) != *(signalMethod_Bento)) {
            ++iter;
            continue;
         }

         if (*(item.slotMethod) != *(slotMethod_Bento)) {
            ++iter;
            continue;
         }

         // connection already exists
         return false;
      }
   }

   sender.addConnection(std::move(signalMethod_Bento), &receiver, std::move(slotMethod_Bento), type, senderListHandle);

   return true;
}

// signal method ptr, slot lambda
template<class Sender, class SignalClass, class ...SignalArgs, class Receiver, class T>
bool connect(const Sender &sender, void (SignalClass::*signalMethod)(SignalArgs...), const Receiver &receiver,
                  T slotLambda, ConnectionKind type, bool uniqueConnection)
{
   // Sender must be the same class as SignalClass and Sender is a child of SignalClass
   Internal::cs_testConnect_SenderSignal<Sender, SignalClass>();

   // compare signal and slot parameter list
   Internal::cs_testConnect_SignalSlotArgs_1<T, SignalArgs...>();

   if (signalMethod == nullptr) {
      throw std::invalid_argument("connect(): Can not connect, signal is null");
   }

   std::unique_ptr<Internal::Bento<void (SignalClass::*)(SignalArgs...)>>
                  signalMethod_Bento(new Internal::Bento<void (SignalClass::*)(SignalArgs...)>(signalMethod));

   std::unique_ptr<Internal::Bento<T>> slotLambda_Bento(new Internal::Bento<T>(slotLambda));

   auto senderListHandle = sender.m_connectList.lock_write();

   if (uniqueConnection) {
      // ensure the connection is not added twice
      auto iter = senderListHandle->begin();
      const auto end = senderListHandle->end();

      while (iter != end) {
         const auto &item = *iter;

         if (item.receiver != &receiver) {
            ++iter;
            continue;
         }

         if (*(item.signalMethod) != *(signalMethod_Bento)) {
            ++iter;
            continue;
         }

         // unable to test if the passed slotLambda = slotLambda_Bento

         // connection already exists
         return false;
      }
   }

   sender.addConnection(std::move(signalMethod_Bento), &receiver, std::move(slotLambda_Bento), type, senderListHandle);

   return true;
}


// signal & slot bento
template<class Sender, class Receiver>
bool connect(const Sender &sender, std::unique_ptr<Internal::BentoAbstract> signalMethod_Bento, const Receiver &receiver,
                  std::unique_ptr<Internal::BentoAbstract> slotMethod_Bento, ConnectionKind type, bool uniqueConnection)
{
   auto senderListHandle = sender.m_connectList.lock_write();

   if (uniqueConnection) {
      // ensure the connection is not added twice
      auto iter = senderListHandle->begin();
      const auto end = senderListHandle->end();

      while (iter != end) {
         const auto &item = *iter;

         if (item.receiver != &receiver) {
            ++iter;
            continue;
         }

         if (*(item.signalMethod) != *(signalMethod_Bento)) {
            ++iter;
            continue;
         }

         if (*(item.slotMethod) != *(slotMethod_Bento)) {
            ++iter;
            continue;
         }

         // connection already exists
         return false;
      }
   }

   sender.addConnection(std::move(signalMethod_Bento), &receiver, std::move(slotMethod_Bento), type, senderListHandle);

   return true;
}

// signal & slot method ptr
template<class Sender, class SignalClass, class ...SignalArgs, class Receiver, class SlotClass, class ...SlotArgs, class SlotReturn>
bool disconnect(const Sender &sender, void (SignalClass::*signalMethod)(SignalArgs...), const Receiver &receiver,
                  SlotReturn (SlotClass::*slotMethod)(SlotArgs...))
{
   // Sender must be the same class as SignalClass and Sender is a child of SignalClass
   Internal::cs_testConnect_SenderSignal<Sender, SignalClass>();

   // Receiver must be the same class as SlotClass and Receiver is a child of SlotClass
   Internal::cs_testConnect_ReceiverSlot<SlotClass, Receiver>();

   // signal & slot arguments do not agree
   Internal::cs_testConnect_SignalSlotArgs_2< void (*)(SignalArgs...), void (*)(SlotArgs...) >();

   Internal::Bento<void (SignalClass::*)(SignalArgs...)> signalMethod_Bento(signalMethod);
   Internal::Bento<SlotReturn (SlotClass::*)(SlotArgs...)> slotMethod_Bento(slotMethod);

   if (! internal_disconnect(sender, &signalMethod_Bento, &receiver, &slotMethod_Bento)) {
      return false;
   }

   return true;
}

// signal method ptr, slot lambda or function ptr
template<class Sender, class SignalClass, class ...SignalArgs, class Receiver, class T>
bool disconnect(const Sender &sender, void (SignalClass::*signalMethod)(SignalArgs...), const Receiver &receiver, T slotMethod)
{
   // lambda, compile error
   static_assert(std::is_convertible<decltype(slotMethod == slotMethod), bool>::value,
                 "disconnect():  Slot argument invalid or calling disconnect using a lambda" );

   // function ptr
   Internal::Bento<void (SignalClass::*)(SignalArgs...)> signalMethod_Bento(signalMethod);
   Internal::Bento<T> slotMethod_Bento(slotMethod);

   if (! internal_disconnect(sender, &signalMethod_Bento, &receiver, &slotMethod_Bento)) {
      return false;
   }

   return true;
}

// signal & slot bento objects
template<class Sender, class Receiver>
bool internal_disconnect(const Sender &sender, const Internal::BentoAbstract *signalBento,
                  const Receiver *receiver, const Internal::BentoAbstract *slotBento)
{
   bool retval = false;
   auto senderListHandle = sender.m_connectList.lock_write();

   for (auto iter = senderListHandle->begin(); iter != senderListHandle->end(); ++iter) {
      const SignalBase::ConnectStruct &temp = *iter;

      bool isMatch = false;

      if (signalBento == nullptr && receiver == nullptr) {
         // delete all connections in Sender
         isMatch = true;

      } else if (receiver != nullptr)  {

         if (receiver == temp.receiver) {

            if (signalBento == nullptr && (slotBento == nullptr || *slotBento == *temp.slotMethod)) {
               isMatch = true;

            } else if (signalBento != nullptr && *signalBento == *temp.signalMethod && (slotBento == nullptr ||
                       *slotBento == *temp.slotMethod)) {
               isMatch = true;
            }
         }

      } else if (signalBento != nullptr) {
         // receiver must be null therefore slot is null

         if (*signalBento == *temp.signalMethod) {
            isMatch = true;
         }
      }

      if (isMatch)  {
         // delete possible sender in the receiver
         retval = true;

         // lock temp.receiver and erase
         auto receiverListHandle = temp.receiver->m_possibleSenders.lock_write();
         receiverListHandle->erase(find(receiverListHandle->begin(), receiverListHandle->end(), &sender));

         // delete connection in sender
         senderListHandle->erase(iter);
      }
   }

   return retval;
}

}  // namespace

// method pointer cast used to resolve ambiguous method overloading for signals and slots

// 1
template<class... Args>
class cs_mp_cast_internal
{
 public:
   template<class className>
   constexpr auto operator()(void (className::*methodPtr)(Args ...)) const
      -> decltype(methodPtr)
   {
      return methodPtr;
   }
};

template<class... Args>
constexpr cs_mp_cast_internal<Args...> cs_mp_cast()
{
   return cs_mp_cast_internal<Args...>();
}

// 2
template<class... Args>
class cs_cmp_cast_internal
{
 public:
   template<class className>
   constexpr auto operator()(void (className::*methodPtr)(Args ...) const) const
      -> decltype(methodPtr)
   {
      return methodPtr;
   }
};

template<class... Args>
constexpr cs_cmp_cast_internal<Args...> cs_cmp_cast()
{
   return cs_cmp_cast_internal<Args...>();
}

#endif
