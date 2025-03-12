# PiSAR: Search and Rescue Robot

**PiSAR** is a robot designed to autonomously navigate a course, use computer vision to detect and rescue a LEGO person, and return home. This robot was designed for the University of Waterloo MTE 380 Design Workshop project.


The system consists of two main components:

- **MCP (Master Control Program, Raspberry Pi)**: Handles vision processing, state management, and high-level decision-making.
- **driveunit (Microcontroller)**: Controls motor actions and responds to commands sent by the Raspberry Pi.

This repository contains all the software for controlling the robot, divided into two primary folders:
- **`driveunit`**: Microcontroller firmware for motor control.
- **`mcp`**: Raspberry Pi code for vision processing and state machine.
