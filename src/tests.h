/*
 * This file is part of the c't-Bot teensy framework.
 * Copyright (c) 2018 Timo Sandmann
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file    tests.h
 * @brief   Test classes implementing tasks to test functionality
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#pragma once

#include <cstdint>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>


namespace ctbot {

class CtBot;
class Ena;

/**
 * @brief Namespace for all test classes
 */
namespace tests {

/**
 * @brief Teensy onboard led blink test
 *
 * @startuml{BlinkTest.png}
 *  !include tests.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class BlinkTest {
protected:
    static constexpr uint16_t TASK_PERIOD_MS { 500 }; /**< Scheduling period of task in ms */

    CtBot& ctbot_; /**< Reference to CtBot instance */
    bool state_;

    /* disable copy/move */
    BlinkTest(const BlinkTest&) = delete;
    void operator=(const BlinkTest&) = delete;
    BlinkTest(BlinkTest&&) = delete;

    /**
     * @brief LED blink test task implementation
     * @note This method is called every TASK_PERIOD_MS ms
     *
     * @startuml{BlinkTest_run.png}
     *  activate BlinkTest
     *  loop every TASK_PERIOD_MS ms
     *    BlinkTest -> BlinkTest: digitalWrite(LED_BUILTIN, state);
     *    BlinkTest -> BlinkTest: toggle(state)
     *  end
     *  deactivate BlinkTest
     * @enduml
     */
    void run();

public:
    /**
     * @brief Constructor, creates the task, that implements the actual functionality
     * @param[in] ctbot: Reference to CtBot instance
     */
    BlinkTest(CtBot& ctbot);

    /**
     * @brief Destructor to destroy task
     * @note Never called in current setup
     */
    ~BlinkTest() = default;
};


/**
 * @brief Led test as simple chaser light with the leds
 *
 * @startuml{LedTest.png}
 *  !include tests.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class LedTest {
protected:
    static constexpr uint16_t TASK_PERIOD_MS { 125 }; /**< Scheduling period of task in ms */

    CtBot& ctbot_; /**< Reference to CtBot instance */

    /* disable copy/move */
    LedTest(const LedTest&) = delete;
    void operator=(const LedTest&) = delete;
    LedTest(LedTest&&) = delete;

    /**
     * @brief LED test task implementation
     * @note This method is called every TASK_PERIOD_MS ms
     *
     * @startuml{LedTest_run.png}
     *  activate LedTest
     *  loop every TASK_PERIOD_MS ms
     *    LedTest -> Leds: off(1 << i)
     *    activate Leds
     *    LedTest <-- Leds
     *    deactivate Leds
     *
     *    LedTest -> LedTest: increment(i)
     *    LedTest -> LedTest: mod(i, 8)
     *
     *    LedTest -> Leds: on(1 << i)
     *    activate Leds
     *    LedTest <-- Leds
     *    deactivate Leds
     *  end
     *  deactivate LedTest
     * @enduml
     */
    void run();

public:
    /**
     * @brief Constructor, creates the task, that implements the actual functionality
     * @param[in] ctbot: Reference to CtBot instance
     */
    LedTest(CtBot& ctbot);

    /**
     * @brief Destructor to destroy task
     * @note Never called in current setup
     */
    ~LedTest() = default;
};


/**
 * @brief Display test
 *
 * @startuml{LcdTest.png}
 *  !include tests.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class LcdTest {
protected:
    static constexpr uint16_t TASK_PERIOD_MS { 100 }; /**< Scheduling period of task in ms */

    CtBot& ctbot_; /**< Reference to CtBot instance */
    uint32_t x_;

    /* disable copy/move */
    LcdTest(const LcdTest&) = delete;
    void operator=(const LcdTest&) = delete;
    LcdTest(LcdTest&&) = delete;

    /**
     * @brief LCD test task implementation
     * @note This method is called every TASK_PERIOD_MS ms
     *
     * @startuml{LcdTest_run.png}
     *  activate LcdTest
     *  loop every TASK_PERIOD_MS ms
     *    LcdTest -> Display: set_cursor((x % 80) / 20 + 1, x % 20 + 1)
     *    activate Display
     *    LcdTest <-- Display
     *    deactivate Display
     *
     *    LcdTest -> Display: print(' ')
     *    activate Display
     *    LcdTest <-- Display
     *    deactivate Display
     *
     *    LcdTest -> LcdTest: increment(x)
     *
     *    LcdTest -> Display: set_cursor((x % 80) / 20 + 1, x % 20 + 1)
     *    activate Display
     *    LcdTest <-- Display
     *    deactivate Display
     *
     *    LcdTest -> Display: print('*')
     *    activate Display
     *    LcdTest <-- Display
     *    deactivate Display
     *
     *    LcdTest -> Display: set_cursor(((x + 20) % 80) / 20 + 1, 1)
     *    activate Display
     *    LcdTest <-- Display
     *    deactivate Display
     *
     *    LcdTest -> Display: print("Hello World :-)")
     *    activate Display
     *    LcdTest <-- Display
     *    deactivate Display
     *
     *    LcdTest -> Display: set_cursor((x + 40) % 80) / 20 + 1, 1)
     *    activate Display
     *    LcdTest <-- Display
     *    deactivate Display
     *
     *    LcdTest -> Display: printf("%5u", x)
     *    activate Display
     *    LcdTest <-- Display
     *    deactivate Display
     *
     *  end
     *  deactivate LcdTest
     * @enduml
     */
    void run();

public:
    /**
     * @brief Constructor, creates the task, that implements the actual functionality
     * @param[in] ctbot: Reference to CtBot instance
     */
    LcdTest(CtBot& ctbot);

    /**
     * @brief Destructor to destroy task
     * @note Never called in current setup
     */
    ~LcdTest() = default;
};


/**
 * @brief Ena test
 *
 * @startuml{EnaTest.png}
 *  !include tests.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class EnaTest {
protected:
    static constexpr uint16_t TASK_PERIOD_MS { 1000 }; /**< Scheduling period of task in ms */

    CtBot& ctbot_; /**< Reference to CtBot instance */
    Ena* p_ena_; /**< Pointer to Ena instance */
    uint8_t ena_idx_;

    /* disable copy/move */
    EnaTest(const EnaTest&) = delete;
    void operator=(const EnaTest&) = delete;
    EnaTest(EnaTest&&) = delete;

    /**
     * @brief ENA test task implementation
     * @note This method is called every TASK_PERIOD_MS ms
     *
     * @startuml{EnaTest_run.png}
     *  activate EnaTest
     *  loop every TASK_PERIOD_MS ms
     *    EnaTest -> Ena: set(1 << i)
     *    activate Ena
     *    EnaTest <-- Ena
     *    deactivate Ena
     *
     *    EnaTest -> EnaTest: increment(i)
     *    EnaTest -> EnaTest: mod(i, 8)
     *  end
     *  deactivate EnaTest
     * @enduml
     */
    void run();

public:
    /**
     * @brief Constructor, creates the task, that implements the actual functionality
     * @param[in] ctbot: Reference to CtBot instance
     */
    EnaTest(CtBot& ctbot);

    /**
     * @brief Destructor to destroy task
     * @note Never called in current setup
     */
    ~EnaTest() = default;
};


/**
 * @brief Sensor data display
 *
 * @startuml{SensorLcdTest.png}
 *  !include tests.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class SensorLcdTest {
protected:
    static constexpr uint16_t TASK_PERIOD_MS { 50 }; /**< Scheduling period of task in ms */

    CtBot& ctbot_; /**< Reference to CtBot instance */

    /**
     * @brief Sensor LCD test task implementation
     * @note This method is called every TASK_PERIOD_MS ms
     *
     * @startuml{SensorLcdTest_run.png}
     *  loop every TASK_PERIOD_MS ms
     *    activate SensorLcdTest
     *
     *    SensorLcdTest -> CtBot: get_senssors()
     *    activate CtBot
     *    SensorLcdTest <-- CtBot
     *    deactivate CtBot
     *
     *
     *    SensorLcdTest -> Display: set_cursor(1, 1)
     *    activate Display
     *    SensorLcdTest <-- Display
     *    deactivate Display
     *
     *    SensorLcdTest -> Display: printf("P%03X %03X D=%4d %4d", LDRL, LDRR, DISTL, DISTR)
     *    activate Display
     *    SensorLcdTest <-- Display
     *    deactivate Display
     *
     *    SensorLcdTest -> Display: set_cursor(2, 1)
     *    activate Display
     *    SensorLcdTest <-- Display
     *    deactivate Display
     *
     *    SensorLcdTest -> Display: printf("B=%03X %03X L=%03X %03X", BORDERL, BORDRR, LINEL, LINER)
     *    activate Display
     *    SensorLcdTest <-- Display
     *    deactivate Display
     *
     *    SensorLcdTest -> Display: set_cursor(3, 1)
     *    activate Display
     *    SensorLcdTest <-- Display
     *    deactivate Display
     *
     *    SensorLcdTest -> Display: printf("R=%2d %2d F=%d K=%d T=%d", ENCL, ENCR, ERROR, SHUTTER, TRANS)
     *    activate Display
     *    SensorLcdTest <-- Display
     *    deactivate Display
     *
     *    SensorLcdTest -> Display: set_cursor(4, 1)
     *    activate Display
     *    SensorLcdTest <-- Display
     *    deactivate Display
     *
     *    SensorLcdTest -> Display: printf("S=%4d %4d", SPEEDL, SPEEDR)
     *    activate Display
     *    SensorLcdTest <-- Display
     *    deactivate Display
     *  end
     *  deactivate SensorLcdTest
     * @enduml
     */
    void run();

public:
    /**
     * @brief Constructor, creates the task, that implements the actual functionality
     * @param[in] ctbot: Reference to CtBot instance
     */
    SensorLcdTest(CtBot& ctbot);

    /**
     * @brief Destructor to destroy task
     * @note Never called in current setup
     */
    ~SensorLcdTest() = default;
};


/**
 * @brief Task wait on condition test
 *
 * @startuml{TaskWaitTest.png}
 *  !include tests.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class TaskWaitTest {
protected:
    static constexpr uint16_t TASK_PERIOD_MS { 100 }; /**< Scheduling period of tasks in ms */

    CtBot& ctbot_; /**< Reference to CtBot instance */
    std::mutex m1_, m2_;
    std::condition_variable cv1_, cv2_;
    std::thread *p_thr1_, *p_thr2_;
    std::atomic<bool> running_;

    /* disable copy/move */
    TaskWaitTest(const TaskWaitTest&) = delete;
    void operator=(const TaskWaitTest&) = delete;
    TaskWaitTest(TaskWaitTest&&) = delete;

public:
    /**
     * @brief Constructor, creates the tasks, that implement the actual functionality
     * @param[in] ctbot: Reference to CtBot instance
     */
    TaskWaitTest(CtBot& ctbot);

    /**
     * @brief Destructor to destroy tasks
     * @note Never called in current setup
     */
    ~TaskWaitTest();

    void stop() {
        running_ = false;
    }
};

} // namespace tests
} // namespace ctbot
