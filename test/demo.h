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

#ifndef DEMO_H
#define DEMO_H

#include <deque>
#include <condition_variable>

#include <cs_signal.h>
#include <cs_slot.h>

class Demo_PushButton : public CsSignal::SignalBase
{
 public:
   SIGNAL_1(void pressed())
   SIGNAL_2(pressed)

   SIGNAL_1(void toggled(bool checked))
   SIGNAL_2(toggled, checked)
};

class Demo_Receiver : public CsSignal::SlotBase
{
 public:
   Demo_Receiver() = default;

   void toggled(bool onOff);
   void methodPressed();
   void threadPressed();

   template<class T>
   void templatePressed();

   int m_slotPressed = 0;
   bool m_slotToggled = false;

   std::deque<CsSignal::PendingSlot> *m_array = nullptr;
   std::mutex *m_mutex = nullptr;
   std::condition_variable *m_alarm = nullptr;

 private:
   void queueSlot(CsSignal::PendingSlot data, CsSignal::ConnectionKind type) override;
};

template<class T>
void Demo_Receiver::templatePressed()
{
   ++m_slotPressed;
}

#endif
