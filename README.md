# STM32F407 Timers Bare Metal Implementation.

## Introduction

A simple implementation of a General Purpose Timer driver on STM32F407.

This driver is written without using any kind of HAL or LL libraries. Completely bare-metal code. This project was developed after finishing Udemy's course [Mastering Microcontroller and Embedded Driver Development](https://www.udemy.com/course/mastering-microcontroller-with-peripheral-driver-development). The purpose of the project is to practice the concepts learned during the course.

Some test applications are developed to test the drivers using the built-in LEDs on the [STM32F407G-DISC1 discovery kit](https://www.st.com/en/evaluation-tools/stm32f4discovery.html).

This project also has the purpose of learning basic concepts related to version control, Git and GitHub.

## Results

The drivers are tested by implementing a simple "breathing effect" on an LED by varying the PWM duty cycle smoothly:

https://github.com/marqueztronics/STM32F407_Timers_Bare_Metal/assets/156628498/4670af2d-5f8d-4612-b62b-055dcd4929f4

### Notes
- The GPIO drivers are implemented by the course instructor, not by me. I implemented the main app, some necessary macros and the General Purpose Timer driver.
- The driver does not implement all features of the General Purpose Timer. Implemented features target TIM2 to TIM5. Other TIMx may not work with this driver.


