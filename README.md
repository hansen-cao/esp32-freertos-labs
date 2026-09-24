# ESP32 FreeRTOS Labs

A collection of C exercises exploring FreeRTOS task coordination and peripheral interfacing on ESP32 using ESP-IDF. These labs document my practice with queues, mutexes, binary semaphores, ADC sampling, UART communication, and dynamic memory.

The examples range from individual RTOS concepts to a small UART-controlled ADC and LED application. They are learning exercises under development, with known issues documented below.

## Lab overview

The numbers below follow the original example order. Each lab is a separate application with its own `app_main()`.

| Lab | Exercise | Concepts practiced |
| --- | --- | --- |
| 01 | ADC-controlled LED timing | ADC one-shot reads, integer queues, producer/consumer tasks, GPIO output |
| 02 | Shared ADC with a mutex | Mutual exclusion and access to a shared peripheral |
| 03 | Structure queue | Passing structured messages by value and blocking on incoming data |
| 04 | Two-channel ADC sampling | Configuring and reading multiple ADC channels, logging normalized readings |
| 05 | Serial message handoff | Serial input, heap allocation, inter-task buffer ownership, heap and stack monitoring |
| 06 | Single-slot producer/consumer | Coordinating simulated package processing with two binary semaphores |
| 07 | UART-controlled ADC and LED | Command parsing, ADC request/response signaling, multiple tasks, LED timing |

Lab 06 uses simulated data and prints results locally; it does not upload data over a network.

## Current status

This repository is a work in progress, not a production firmware package. The examples need the following improvements:

- **Lab 01:** Create the queue before starting either task. Handle queue creation/send failures and decide whether LED timing should use the latest sample or process every queued sample.
- **Lab 02:** Acquire and release the mutex on every loop iteration. Move the task delay inside the loop, after releasing the mutex.
- **Lab 03:** Check task creation and queue-send results, and document the policy for a full queue.
- **Lab 04:** Verify ADC channel-to-pin mappings for the target board and make the ADC resolution consistent with the normalization calculation. The displayed percentages are fractions of the ADC code range, not calibrated physical measurements.
- **Lab 05:** Replace the shared pointer/flag handoff with explicit synchronization and buffer ownership. Correct input reset, length, and termination handling.
- **Lab 06:** Check semaphore and task creation results and document the single-slot handoff behavior.
- **Lab 07:** Check input bounds before writing, synchronize command-buffer ownership and LED timing updates, initialize the LED delay, and avoid discarding pending commands when flushing UART input.

## Hardware and software

- An ESP32 development board supported by the selected ESP-IDF version
- ESP-IDF and its toolchain
- A serial connection for flashing, logs, and UART exercises
- An LED with appropriate current limiting for GPIO exercises
- Suitable analog inputs for ADC exercises

Check the specific board's pinout and electrical limits before wiring. GPIO and ADC channel definitions in the examples must match the selected chip and board. UART settings and console routing may also require adjustment.

## Running a lab

Each `main.c` belongs in its own ESP-IDF application. The files alone are not complete, independently buildable ESP-IDF projects.

1. Create or copy a basic ESP-IDF project for the target board.
2. Use the selected lab as the application's main source file and configure its component dependencies.
3. Check the GPIO, ADC, and UART configuration against the board and wiring.
4. Address the lab's known issues before treating it as a working reference.
5. Build, flash, and monitor using the ESP-IDF tools configured for your board.

Do not compile all seven examples together: each defines an application entry point.

Reproducible releases will need a documented ESP-IDF version, chip/board model, build configuration, wiring, and expected output for each lab.

## Validation goals

- Compare observed task behavior with the intended synchronization sequence.
- Exercise queue-full and slow-consumer conditions.
- Test empty, maximum-length, overlong, and back-to-back serial commands.
- Check ADC endpoint behavior and LED timing.
- Monitor heap and stack usage during repeated operation.

These are planned checks, not claims of completed testing.

## Author

**Wenhan (Hansen) Cao** — Electrical Engineering student at Georgia Tech

[GitHub](https://github.com/Hansen-Cao)
