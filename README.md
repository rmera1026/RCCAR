# RCCAR

Firmware project for an STM32-based RC car system split into two applications:
- Controller firmware that sends drive commands
- Receiver firmware that interprets commands and drives the motors

## Repository Layout

- `RCCAR_Controller/`: Transmitter-side STM32CubeIDE project
- `RCCAR_Receiver/`: Vehicle-side STM32CubeIDE project

Each project contains the usual STM32CubeIDE structure:
- `Core/Inc`, `Core/Src`: Application and HAL integration code
- `Startup/`: Startup assembly
- `Drivers/`: CMSIS and STM32 HAL drivers

## System Behavior

Receiver command handling includes:
- Forward
- Reverse
- Left
- Right
- Stop

Safety behavior:
- Failsafe timeout forces stop if valid commands are not received in time

## Schematics

Schematic drawings are available for both sides of the system:
- Controller: [RCCAR_Controller_Schematic.png](RCCAR_Controller/RCCAR_Controller_Schematic.png)
- Receiver: [RCCAR_Receiver_Schematic.png](RCCAR_Receiver/RCCAR_Receiver_Schematic.png)

## Hardware/Toolchain

- MCU family: STM32F103
- IDE: STM32CubeIDE
- Language: C
- Framework: STM32 HAL + CMSIS

## Build and Flash

1. Open `RCCAR_Controller` in STM32CubeIDE.
2. Build the project (`Project -> Build Project`).
3. Flash to controller board using ST-Link (`Run -> Debug` or `Run -> Run`).
4. Repeat for `RCCAR_Receiver` and flash to receiver board.

Generated artifacts are placed under each project's `Debug/` directory.

## Bring-Up Checklist

1. Power both boards.
2. Verify receiver boots and motors are idle.
3. Send each command from controller and confirm expected motion:
	- Forward
	- Reverse
	- Left
	- Right
	- Stop
4. Turn off controller signal path and verify receiver enters failsafe stop.