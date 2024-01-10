<div style="float: right">
    <img src="https://upload.wikimedia.org/wikipedia/commons/thumb/8/86/Festo_logo.svg/1200px-Festo_logo.svg.png" alt="Festo Logo" width="150">
</div>

# Festo SCARA Robot Control System

This C++ project leverages the [Festo Library EtherCAT](https://github.com/Jkachoura/CMMT-EtherCAT/) for precise control of a SCARA robot at Festo. The primary goal is to showcase the potential of controlling Festo machines directly from a PC, bypassing the traditional Programmable Logic Controller (PLC) setup.

**Note** This project is specifically designed for a SCARA configuration in Festo Delft.

## Table of Contents

- [Features](#features)
- [Dependencies](#dependencies)
- [Usage](#usage)

## Features

- **Inverse Kinematics**: Utilizes inverse kinematics to move the SCARA robot to the desired end effector position.
- **Relative Gripper Angle Calculation**: Calculates the relative gripper angle for efficient object manipulation.
- **Air Pressure Control**: Manages the air pressure to enable the gripper for picking up objects.
- **Pressure Sensor Integration**: Utilizes a pressure sensor to ensure the generation of a vacuum for object grasping.
- **Festo Camera Implementation**: Integrates a Festo camera to capture images of the search field.
- **Data Handling for Camera Output**: Implements data handling mechanisms for processing information obtained from the Festo camera.

## Dependencies

- Festo Library EtherCAT: [Link to the Library](https://github.com/Jkachoura/CMMT-EtherCAT/)
- 2 Ethernet Cables

**Note** For a guide on setting up and installing the Festo Library, please refer to the repository of that library.

## Usage

1. Physically connect one Ethernet Cable to the servomaster and another to the camera.

2. Use slaveinfo.exe to configure the device name and update it in main.cpp within the master folder:

```C++
char ifaceName[] = "\\Device\\NPF_{DEA85026-34BA-4C8B-9840-A3CE7793A348}";
Master ecMaster(ifaceName, 8000);
```

3. Ensure that you press the blue button (reset button) before initiating the program.

4. Select master.exe and press the "Run" button in Visual Studio to start the execution.