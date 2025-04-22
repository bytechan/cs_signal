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

#define CS_STRING_ALLOW_UNSAFE

#include <cs_signal.h>
#include <demo.h>

#include <atomic>

#include <cs_catch2.h>

TEST_CASE("CsSignal traits", "[cs_signal]") {
  REQUIRE(std::is_copy_constructible_v<CsSignal::SignalBase> == true);
  REQUIRE(std::is_move_constructible_v<CsSignal::SignalBase> == true);

  REQUIRE(std::is_copy_assignable_v<CsSignal::SignalBase> == false);
  REQUIRE(std::is_move_assignable_v<CsSignal::SignalBase> == false);

  REQUIRE(std::has_virtual_destructor_v<CsSignal::SignalBase> == true);
}

TEST_CASE("CsSignal connect_method", "[cs_signal]") {
  Demo_PushButton okButton;

  Demo_Receiver objReceiver = Demo_Receiver{};

  connect(&okButton, &Demo_PushButton::pressed, &objReceiver,
          &Demo_Receiver::methodPressed);

  // ensure slot has not been accidentally called
  REQUIRE(objReceiver.m_slotPressed == 0);

  // call the signal
  okButton.pressed();

  // ensure slot has been called once
  REQUIRE(objReceiver.m_slotPressed == 1);
}

TEST_CASE("CsSignal connect_lambda", "[cs_signal]") {
  int slotPressed = 0;

  Demo_PushButton okButton;

  Demo_Receiver objReceiver = Demo_Receiver{};

  connect(&okButton, &Demo_PushButton::pressed, &objReceiver,
          [&slotPressed]() { ++slotPressed; });

  // ensure slot has not been called
  REQUIRE(slotPressed == 0);

  okButton.pressed();

  // ensure slot has been called once
  REQUIRE(slotPressed == 1);
}

TEST_CASE("CsSignal connect_template", "[cs_signal]") {
  Demo_PushButton okButton;

  Demo_Receiver objReceiver = Demo_Receiver{};

  connect(&okButton, &Demo_PushButton::pressed, &objReceiver,
          &Demo_Receiver::templatePressed<int>,
          CsSignal::ConnectionKind::AutoConnection);

  // ensure slot has not been called
  REQUIRE(objReceiver.m_slotPressed == 0);

  okButton.pressed();

  // ensure slot has been called once
  REQUIRE(objReceiver.m_slotPressed == 1);
}

TEST_CASE("CsSignal connect_toggled", "[cs_signal]") {
  Demo_PushButton okButton;

  Demo_Receiver objReceiver = Demo_Receiver{};

  connect(&okButton, &Demo_PushButton::toggled, &objReceiver,
          &Demo_Receiver::toggled, CsSignal::ConnectionKind::AutoConnection);

  // ensure slot has not been called
  REQUIRE(objReceiver.m_slotToggled == false);

  okButton.toggled(true);

  // ensure slot has been called once
  REQUIRE(objReceiver.m_slotToggled == true);
}

static void callBack(std::atomic<bool> &running,
                     std::deque<CsSignal::PendingSlot> &array,
                     std::mutex &mutex, std::condition_variable &alarm) {
  while (true) {
    std::unique_lock<std::mutex> lock(mutex);

    if (!array.empty()) {
      auto data = std::move(array.front());
      array.pop_front();
      lock.unlock();

      // call the slot
      data();
      continue;

    } else if (!running) {
      break;
    }

    alarm.wait(lock);
  }
}

TEST_CASE("CsSignal discontent", "[cs_signal]") {
  Demo_PushButton okButton;

  Demo_Receiver objReceiver = Demo_Receiver{};

  connect(&okButton, &Demo_PushButton::pressed, &objReceiver,
          &Demo_Receiver::methodPressed,
          CsSignal::ConnectionKind::AutoConnection);

  // ensure slot has not been called
  REQUIRE(objReceiver.m_slotPressed == 0);

  okButton.pressed();

  disconnect(&okButton, &Demo_PushButton::pressed, &objReceiver,
             &Demo_Receiver::methodPressed);

  okButton.pressed();

  // ensure slot has been called once
  REQUIRE(objReceiver.m_slotPressed == 1);
}

TEST_CASE("CsSignal thread", "[cs_signal]") {
  // set up threads
  std::atomic<bool> running;
  running = true;

  std::deque<CsSignal::PendingSlot> array;
  std::mutex mutex;
  std::condition_variable alarm;

  std::thread thread1(callBack, std::ref(running), std::ref(array),
                      std::ref(mutex), std::ref(alarm));

  Demo_PushButton okButton;

  Demo_Receiver objReceiver;
  objReceiver.m_array = &array;
  objReceiver.m_mutex = &mutex;
  objReceiver.m_alarm = &alarm;

  connect(&okButton, &Demo_PushButton::pressed, &objReceiver,
          &Demo_Receiver::threadPressed,
          CsSignal::ConnectionKind::QueuedConnection);

  // ensure slot has not been called
  REQUIRE(objReceiver.m_slotPressed == 0);

  okButton.pressed();

  running = false;

  {
    std::lock_guard tmpGuard(mutex);

    // wake up the thread
    alarm.notify_one();
  }

  thread1.join();

  // ensure slot has been called
  REQUIRE(objReceiver.m_slotPressed == 1);
}

TEST_CASE("CsSignal disconnect_all", "[cs_signal]") {
  Demo_PushButton okButton;
  Demo_Receiver receiver1;
  Demo_Receiver receiver2;
  int lambdaCount = 0;

  // Connect multiple slots to the same signal
  connect(&okButton, &Demo_PushButton::pressed, &receiver1,
          &Demo_Receiver::methodPressed,
          CsSignal::ConnectionKind::AutoConnection);
  connect(&okButton, &Demo_PushButton::pressed, &receiver2,
          &Demo_Receiver::methodPressed,
          CsSignal::ConnectionKind::AutoConnection);
  connect(&okButton, &Demo_PushButton::pressed,
          [&lambdaCount]() { ++lambdaCount; });

  // Verify initial state
  REQUIRE(receiver1.m_slotPressed == 0);
  REQUIRE(receiver2.m_slotPressed == 0);
  REQUIRE(lambdaCount == 0);

  // Emit signal to verify connections work
  okButton.pressed();

  // Verify all slots were called
  REQUIRE(receiver1.m_slotPressed == 1);
  REQUIRE(receiver2.m_slotPressed == 1);
  REQUIRE(lambdaCount == 1);

  // Disconnect all slots from the pressed signal
  bool disconnected = disconnect(&okButton, &Demo_PushButton::pressed);
  REQUIRE(disconnected == true);

  // Emit signal again
  okButton.pressed();

  // Verify no slots were called this time
  REQUIRE(receiver1.m_slotPressed == 1); // Still 1 from before
  REQUIRE(receiver2.m_slotPressed == 1); // Still 1 from before
  REQUIRE(lambdaCount == 1);             // Still 1 from before

  // Try disconnecting again, should return false since no connections exist
  disconnected = disconnect(&okButton, &Demo_PushButton::pressed);
  REQUIRE(disconnected == false);

  // Test that other signals are unaffected
  connect(&okButton, &Demo_PushButton::toggled, &receiver1,
          &Demo_Receiver::toggled, CsSignal::ConnectionKind::AutoConnection);
  okButton.toggled(true);
  REQUIRE(receiver1.m_slotToggled == true);
}

TEST_CASE("CsSignal unique_connection", "[cs_signal]") {
  Demo_PushButton button;
  Demo_Receiver receiver;

  SECTION("Normal method connections") {
    // First connection should succeed
    bool connected = connect(&button, &Demo_PushButton::pressed, &receiver,
                             &Demo_Receiver::methodPressed,
                             CsSignal::ConnectionKind::AutoConnection,
                             true); // uniqueConnection = true
    REQUIRE(connected == true);

    // Second identical connection should fail
    connected = connect(&button, &Demo_PushButton::pressed, &receiver,
                        &Demo_Receiver::methodPressed,
                        CsSignal::ConnectionKind::AutoConnection,
                        true); // uniqueConnection = true
    REQUIRE(connected == false);

    // Signal should only trigger once
    button.pressed();
    REQUIRE(receiver.m_slotPressed == 1);
  }

  SECTION("Non-unique connections") {
    // First connection
    bool connected = connect(&button, &Demo_PushButton::pressed, &receiver,
                             &Demo_Receiver::methodPressed,
                             CsSignal::ConnectionKind::AutoConnection,
                             false); // uniqueConnection = false
    REQUIRE(connected == true);

    // Second identical connection should succeed
    connected = connect(&button, &Demo_PushButton::pressed, &receiver,
                        &Demo_Receiver::methodPressed,
                        CsSignal::ConnectionKind::AutoConnection,
                        false); // uniqueConnection = false
    REQUIRE(connected == true);

    // Signal should trigger twice
    button.pressed();
    REQUIRE(receiver.m_slotPressed == 2);
  }

  SECTION("Lambda connections") {
    int lambdaCount = 0;
    auto lambda = [&lambdaCount]() { ++lambdaCount; };

    // First lambda connection should succeed
    bool connected = connect(&button, &Demo_PushButton::pressed, lambda);
    REQUIRE(connected == true);

    // Second identical lambda connection should fail
    connected = connect(&button, &Demo_PushButton::pressed, lambda);
    REQUIRE(connected == true);

    // Signal should trigger twice 
    button.pressed();
    REQUIRE(lambdaCount == 2);
  }

  SECTION("Mixed connections") {
    int lambdaCount = 0;
    auto lambda = [&lambdaCount]() { ++lambdaCount; };

    // Method connection
    bool connected = connect(&button, &Demo_PushButton::pressed, &receiver,
                             &Demo_Receiver::methodPressed,
                             CsSignal::ConnectionKind::AutoConnection,
                             true); // uniqueConnection = true
    REQUIRE(connected == true);

    // Lambda connection should succeed (different type)
    connected = connect(&button, &Demo_PushButton::pressed, lambda);
    REQUIRE(connected == true);

    // Signal should trigger both
    button.pressed();
    REQUIRE(receiver.m_slotPressed == 1);
    REQUIRE(lambdaCount == 1);
  }
}
