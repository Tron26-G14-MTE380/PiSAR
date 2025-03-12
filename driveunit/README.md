# PiSAR driveunit

This folder contains the firmware for the microcontroller (Raspberry Pi Pico) that controls the robot's motors and performs real-time motor actions based on commands received from the Raspberry Pi (mcp). The firmware listens for control signals via SPI from the Raspberry Pi, processes them, and adjusts the robot’s movement accordingly.

## Overview

The **driveunit** is responsible for the following tasks:
- Receiving motor control commands (e.g., trajectory) from the Raspberry Pi via SPI.
- Controlling motor actions to perform tasks such as moving forward, backward, and turning.
- Managing the robot's low-level movement, including control of motors.
