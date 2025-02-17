# FRC 1732 - Operatator Panel LED/Button Support

This page describes how the button LEDs can be controled to represent the level and position last selected.
---
## Hardware and Software Overview

Default button support for LED control of the arcade buttons is either always on or on when pressed.  This is fucntionally built into the wiring of the joystick board.  However, the LEDs on the buttons have their own connectors that need not be part of the joystick wiring.  Instead, button state can be mantained outside of joystick control and a separate mini control system can be responsible for powering the button LEDs.
<p>
Solution:<p>

1. A robot subsystem is tracking button state. Likely options could be the `RGBSubsystem`.
1. Button state is published to Network tables as a parsable String such as "L3P07" for Level 3 Position 7.
1. Simple application running on driver station that connects to network tables to read this string value and write it out to a USB Serial interface. A simple java console app could fulfill this need as well as many other options.
1. An arduino with a USB connection back to the driver station will be able to establish a serial connection and receive the text write from the application.
1. The arduino will parse the string and set outputs to control specific button LEDs.

### Operator Controls
- **Button panel** that includes various switches and buttons for advanced controls.  Notably this include 4 level buttons and 12 position buttons for a total of 16 buttons that we want to control the lighting.

![My Image](images/operatorPanel.png)

- **Driver Station Application** can be a simple console application that understands the network tables and Serial USB connection.

- **Arduino** will use the Serial interface to receive text updates from the driver station computer.  Likewise the arduino will be able to use teh same Serial interface to return acknowledgement messages.  Controlling 16 buttons would require 16 IO pins and while larger Arduino form factors, the smaller solution is a 4 to 16 decoder chip like the CD54HC4514 chip.  Using four outputs plus another for a latch, with the decode chip, all 16 button LEDs can be controlled.
