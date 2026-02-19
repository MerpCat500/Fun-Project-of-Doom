/**
 * @file Threading.hpp
 * @author Andrew Hilton (2131H)
 * @brief Memory Safe Multithreaded Task
 * @version 0.1
 * @date 2026-02-18
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <sys/types.h>

#include <functional>
#include <memory>
#include <string>
#include <type_traits>

#include "pros/rtos.hpp"

// TODO: MARKED AS UNTESTED!
template <typename T>
class Task
{
 private:
  // Reference to data for the task to operate on, specifically a class
  // that inherits from std::enable_shared_from_this<T>
  T* data;

  // PROS Rtos task object
  pros::Task task = pros::Task([]() {});

  // Delay between task function executions, in milliseconds
  uint32_t delayMs;

  // Task function to execute in the task, should capture only this as any
  // other data will not be protected by the weak pointer safety mechanism
  std::function<void(void)> taskFunction;

  // Name of the task for debugging purposes
  std::string taskName;

  // Whether the task should continue running, set to false to signal the
  // task to exit cleanly
  bool validThread = true;

 public:
  Task() : validThread(false) {}

  Task(
      T* data,
      std::function<void(void)> taskFunction,
      std::string taskName,
      uint32_t delayMs = 10)
      : data(data),
        taskFunction(taskFunction),
        delayMs(delayMs),
        taskName(taskName),
        validThread(true)
  {
  }

  void start()
  {
    // Require that T inherits from std::enable_shared_from_this<T> to
    // ensure that we can get a weak pointer reference to the underlying
    // data for safe multithreading
    static_assert(
        std::is_base_of_v<std::enable_shared_from_this<T>, T>,
        "Task<T> requires T to inherit from"
        " std::enable_shared_from_this<T>");

    // Check that the thread is valid before starting (IE, Not default
    // constructed)
    if (validThread)
    {
      // Get a weak pointer reference to the underlying data for the task
      std::weak_ptr<T> self = data->weak_from_this();

      // Start the PROS task with a lambda that captures the weak pointer
      task = pros::Task(
          [&]() {
            bool valid = true;

            // While the thread is marked as valid
            while (valid)
            {
              if (auto s = self.lock())
              {
                valid = validThread;  // Check if the thread is still valid
                taskFunction();       // Execute the task function
              }
              else
              {
                valid = false;
              }  // If the weak pointer can't be locked, the underlying
                 // data has been destroyed, so exit the thread cleanly

              // Delay to not hog the CPU, and to allow other tasks to run.
              pros::delay(delayMs);
            }
          },
          taskName.c_str());
    }
  }
  void remove() { validThread = false; }

  ~Task() { validThread = false; }
};