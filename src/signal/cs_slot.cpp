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

#include "cs_signal.h"
#include "cs_slot.h"

CsSignal::SlotBase::SlotBase()
{
}

CsSignal::SlotBase::SlotBase(const SlotBase &)
{
}

CsSignal::SlotBase::~SlotBase()
{
   try {
      // clean up possible sender connections
      auto receiverListHandle = m_possibleSenders.lock_read();
      auto begin = receiverListHandle->begin();
      auto end = receiverListHandle->end();

      for (auto iter = begin; iter != end; ++iter) {
         const SignalBase* sender = *iter;
         auto senderListHandle = sender->m_connectList.lock_write();

         auto connIter = senderListHandle->begin();
         while (connIter != senderListHandle->end()) {
            if (connIter->receiver == this) {
               connIter = senderListHandle->erase(connIter);
            } else {
               ++connIter;
            }
         }
      }

   } catch (...) {
     // least of the worst options
     std::terminate();
   }
}

CsSignal::SignalBase *&CsSignal::SlotBase::get_threadLocal_currentSender()
{
#ifdef __APPLE__
   static __thread CsSignal::SignalBase *threadLocal_currentSender = nullptr;
#else
   static thread_local CsSignal::SignalBase *threadLocal_currentSender = nullptr;
#endif

   return threadLocal_currentSender;
}

bool CsSignal::SlotBase::compareThreads() const
{
   return true;
}

void CsSignal::SlotBase::queueSlot(PendingSlot data, ConnectionKind)
{
   // calls the slot immediately
   data();
}

CsSignal::SignalBase *CsSignal::SlotBase::sender() const
{
   return get_threadLocal_currentSender();
}

std::set<CsSignal::SignalBase *> CsSignal::SlotBase::internal_senderList() const
{
   std::set<SignalBase *> retval;

   auto receiverListHandle = m_possibleSenders.lock_read();
   auto iter = receiverListHandle->begin();
   const auto end = receiverListHandle->end();

   while (iter != end) {
      retval.insert(const_cast<SignalBase *>(*iter));
      ++iter;
   }

   return retval;
}

CsSignal::PendingSlot::PendingSlot(SignalBase *sender, std::unique_ptr<Internal::BentoAbstract> signal_Bento,
                  SlotBase *receiver, std::unique_ptr<Internal::BentoAbstract> slot_Bento,
                  std::unique_ptr<Internal::TeaCupAbstract> teaCup_Data)
   : m_sender(sender), m_signal_Bento(std::move(signal_Bento)), m_receiver(receiver),
     m_slot_Bento(std::move(slot_Bento)), m_teaCup_Data(std::move(teaCup_Data))
{
}

void CsSignal::PendingSlot::operator()() const
{
   // invoke the slot
   m_slot_Bento->invoke(m_receiver, m_teaCup_Data.get());
}

