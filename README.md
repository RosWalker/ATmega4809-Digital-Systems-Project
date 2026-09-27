# ATmega4809-Digital-Systems-Project

# ATmega4809 Rotating Motor Control Project

## Overview

This project was developed using an **ATmega4809 Nano Every** microcontroller and an **OER electronics shield**. The aim was to develop a rotating motor system while gaining practical experience with microcontroller peripherals, timers, interrupts, PWM, UART communication and sensor interfacing.

The project was programmed in **C** and required direct configuration and control of the ATmega4809's internal peripherals.

## Hardware

* ATmega4809 Nano Every
* OER development shield
* DC/rotating motor
* Servo motor
* HC-SR04 ultrasonic sensor
* 555 timer circuits
* LEDs / indicator outputs
* Serial interface

## Key Features

* Motor control using microcontroller timer peripherals
* PWM generation for motor/servo control
* Ultrasonic distance measurement using the HC-SR04
* USART serial communication at **115200 baud**
* Timer-based event scheduling
* ADC measurements and triggering
* Digital input/output control
* Interrupt-driven peripheral operation

## Microcontroller Peripherals

The project made extensive use of the ATmega4809's hardware peripherals, including:

| Peripheral  | Application                   |
| ----------- | ----------------------------- |
| TCA0        | PWM / motor and servo control |
| TCB0        | HC-SR04 timing                |
| TCB1 / TCB2 | 555 timer signal measurement  |
| TCB3        | ADC triggering                |
| USART3      | Serial communication          |
| ADC         | Analogue measurements         |
| RTC/PIT     | Periodic timing               |
| GPIO        | Digital inputs and outputs    |

## Software

The firmware was written in **C** with direct register-level configuration of the ATmega4809 peripherals.

The project involved:

* Initialising microcontroller peripherals
* Configuring timers and counters
* Generating PWM signals
* Measuring timing signals
* Handling interrupts
* Reading sensor data
* Communicating through USART
* Controlling outputs based on sensor/input data

## What I Learned

This project developed my understanding of:

* Embedded C programming
* Microcontroller architecture
* Timers and counters
* PWM generation
* Interrupts
* UART/USART communication
* Sensor interfacing
* ADC operation
* Real-time embedded systems
* Debugging hardware/software interactions

## Project Structure

```text
ATmega4809-Motor-Project/
│
├── src/
│   └── main.c
│
├── README.md
│
└── images/
    ├── project_setup.jpg
    └── motor_setup.jpg
```

## Demonstration

Images and videos of the completed hardware setup can be found in the `images` folder.

## Technologies

**Programming:** C
**Microcontroller:** ATmega4809
**Development Board:** Arduino Nano Every
**Embedded Systems:** Timers, PWM, ADC, USART, Interrupts
**Sensors:** HC-SR04
**Hardware:** OER Shield, motors and 555 timer circuits
