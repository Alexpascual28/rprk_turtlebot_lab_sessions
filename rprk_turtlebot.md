<p align="center">
  <img src="https://raw.githubusercontent.com/PKief/vscode-material-icon-theme/ec559a9f6bfd399b82bb44393651661b08aaf7ba/icons/folder-markdown-open.svg" width="100" alt="project-logo">
</p>
<p align="center">
    <h1 align="center">RPRK Turtlebot Lab Sessions</h1>
</p>
<p align="center">
    <em><code>University of York RPRK lab sessions for MSc Intelligent Robotics ELE00118M Practical Robotics (PRAR). These lab sessions show how a general control "Turtlebot" class to control the RPRK robot is achieved by solving the lab tasks progressively. To see last version of the RPRK class, check rprk_control_class.</code></em>
</p>

<br><!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary><br>

- [ Overview](#-overview)
- [ Features](#-features)
- [ Repository Structure](#-repository-structure)
- [ Modules](#-modules)
- [ Getting Started](#-getting-started)
  - [ Installation](#-installation)
  - [ Usage](#-usage)
  - [ Tests](#-tests)
- [ Project Roadmap](#-project-roadmap)
- [ Contributing](#-contributing)
- [ License](#-license)
- [ Acknowledgments](#-acknowledgments)
</details>
<hr>

##  Overview

<code>The RPRK (Raspberry Pi Robotics Kit) lab sessions are designed for the MSc Intelligent Robotics course at the University of York, specifically for the ELE00118M Practical Robotics module, in the Autumn term. The kit allows students to build a mobile robot equipped with various sensors and actuators managed by a Raspberry Pi Model 4 and an Arduino Nano 33 BLE via the custom Arduino Robotics Board (ARB). These sessions guide students through the fundamentals of robotics, including sensor integration, motor control, and serial communications, culminating in the development of a robot that can navigate and map an environment autonomously.</code>

<code>This project is designed for students and developers interested in the fields of robotics and software engineering, using Raspberry Pi, Arduino, Python, and C++. It contains a set of practical lab sessions that allow hands-on experience with real-world robotic systems programming and control.</code>

<code>The focus is on interfacing and controlling various robotic functionalities such as motor control, sensor integration, and visual processing using OpenCV. The project is structured in a way to guide the user from simple to more complex robotics applications, making it suitable for educational purposes or for hobbyists looking to enhance their robotics skills.</code>

---

##  Directory Description

* **ARB/**: Contains the Arduino library (ARB.cpp, ARB.h) used for managing serial communication on the Arduino side.
* **ARBPi library for Pi Serial comms/**: Contains Python and C++ libraries for handling serial communications on the Raspberry Pi side.
* **BLE Example/**: Example files demonstrating Bluetooth Low Energy (BLE) communications.
* **Lab[1-9]/**: Each folder corresponds to a lab session and contains specific tasks and PDF instructions.
* **Library (Last Version)/**: Contains the final version of the TurtleBot Python class.
* **Showcase/** and **UCAS_showcase/**: Contains code for testing different programs and functionalities of the RPRK kit. These directories include advanced examples that integrate concepts from multiple lab sessions into a single showcase, demonstrating a more complex robot operation scenario.

---

##  Repository Structure

```sh
└── rprk_turtlebot_lab_sessions/
    ├── ARB
    │   ├── ARB.cpp
    │   ├── ARB.h
    │   ├── examples
    │   └── keywords.txt
    ├── ARBPi library for Pi Serial comms
    │   ├── ARBPi.cpp
    │   ├── ARBPi.h
    │   ├── ARBPi.py
    │   ├── libARBPi.so
    │   ├── serialTest.cpp
    │   ├── serialTest.py
    │   └── serialtest
    ├── BLE Example
    │   ├── BLECentral.c
    │   ├── BLECentral.exe
    │   ├── BLECentral.obj
    │   ├── include
    │   ├── libsimpleble-c-static.a
    │   ├── libsimpleble-static.a
    │   ├── libsimplebluez-static.a
    │   ├── libsimpledbus-static.a
    │   ├── simpleble-c-static.lib
    │   └── simpleble-static.lib
    ├── Lab1
    │   ├── autumn_lab_1.pdf
    │   ├── pigpio_example.c
    │   ├── pigpio_example.py
    │   ├── task5.c
    │   └── task6.py
    ├── Lab2
    │   ├── autumn_lab_2.pdf
    │   ├── task1
    │   └── task2
    ├── Lab3-4
    │   ├── autumn_lab_4.pdf
    │   ├── task4
    │   └── task5
    ├── Lab5
    │   ├── ARBPi.cpp
    │   ├── ARBPi.h
    │   ├── ARBPi.py
    │   ├── Speed estimation.xlsx
    │   ├── TurtleBot.py
    │   ├── autumn_lab_5.pdf
    │   ├── encoderMotorWASD
    │   ├── encoderReading
    │   ├── encoderSerial
    │   ├── encoderSpeed
    │   ├── libARBPi.so
    │   ├── odometryControl
    │   ├── task2.py
    │   ├── task2p1.py
    │   └── task3.py
    ├── Lab6
    │   ├── TurtleBot.py
    │   ├── aruco_test.py
    │   ├── autumn_lab_6.pdf
    │   ├── blob.py
    │   ├── blobhd.py
    │   ├── gen_aruco_pdf.py
    │   ├── mask.py
    │   ├── test_camera.py
    │   └── test_camera_fps.py
    ├── Lab7
    │   └── autumn_lab_7.pdf
    ├── Lab8
    │   ├── ARBPi.cpp
    │   ├── ARBPi.h
    │   ├── ARBPi.py
    │   ├── TurtleBot.py
    │   ├── autumn_lab_8.pdf
    │   ├── generalControl
    │   ├── infraredSensor.py
    │   ├── joystickControl.py
    │   ├── libARBPi.so
    │   ├── task2circle.py
    │   ├── task2eight.py
    │   ├── task2line.py
    │   ├── task2rectangle.py
    │   ├── task2square.py
    │   ├── task3.py
    │   └── ultrasoundSensor.py
    ├── Lab9
    │   ├── ARBPi.cpp
    │   ├── ARBPi.h
    │   ├── ARBPi.py
    │   ├── TurtleBot.py
    │   ├── autumn_lab_9.pdf
    │   ├── libARBPi.so
    │   ├── obstacle_test.py
    │   └── speed_conversion_new_test.py
    ├── Library (Last Version)
    │   └── TurtleBot.py
    ├── README.md
    ├── Showcase
    │   ├── ARBPi.cpp
    │   ├── ARBPi.h
    │   ├── ARBPi.py
    │   ├── TurtleBot.py
    │   ├── __pycache__
    │   ├── libARBPi.so
    │   └── showcase.py
    └── UCAS_showcase
        ├── generalControl
        └── initial_testing
```

---

##  Modules

<details closed><summary>All Modules</summary>

<details closed><summary>Library (Last Version)</summary>

| File                                                                                                                             | Summary                         |
| ---                                                                                                                              | ---                             |
| [TurtleBot.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Library (Last Version)/TurtleBot.py) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>Lab5</summary>

| File                                                                                                           | Summary                         |
| ---                                                                                                            | ---                             |
| [ARBPi.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab5/ARBPi.h)           | <code>► INSERT-TEXT-HERE</code> |
| [ARBPi.cpp](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab5/ARBPi.cpp)       | <code>► INSERT-TEXT-HERE</code> |
| [task2p1.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab5/task2p1.py)     | <code>► INSERT-TEXT-HERE</code> |
| [task3.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab5/task3.py)         | <code>► INSERT-TEXT-HERE</code> |
| [TurtleBot.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab5/TurtleBot.py) | <code>► INSERT-TEXT-HERE</code> |
| [ARBPi.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab5/ARBPi.py)         | <code>► INSERT-TEXT-HERE</code> |
| [task2.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab5/task2.py)         | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>Lab5.encoderSpeed</summary>

| File                                                                                                                                | Summary                         |
| ---                                                                                                                                 | ---                             |
| [encoderSpeed.ino](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab5/encoderSpeed/encoderSpeed.ino) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>Lab5.encoderMotorWASD</summary>

| File                                                                                                                                            | Summary                         |
| ---                                                                                                                                             | ---                             |
| [encoderMotorWASD.ino](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab5/encoderMotorWASD/encoderMotorWASD.ino) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>Lab5.odometryControl</summary>

| File                                                                                                                                         | Summary                         |
| ---                                                                                                                                          | ---                             |
| [odometryControl.ino](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab5/odometryControl/odometryControl.ino) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>Lab5.encoderSerial</summary>

| File                                                                                                                                   | Summary                         |
| ---                                                                                                                                    | ---                             |
| [encoderSerial.ino](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab5/encoderSerial/encoderSerial.ino) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>Lab5.encoderReading</summary>

| File                                                                                                                                      | Summary                         |
| ---                                                                                                                                       | ---                             |
| [encoderReading.ino](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab5/encoderReading/encoderReading.ino) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>ARB</summary>

| File                                                                                                          | Summary                         |
| ---                                                                                                           | ---                             |
| [keywords.txt](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/ARB/keywords.txt) | <code>► INSERT-TEXT-HERE</code> |
| [ARB.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/ARB/ARB.h)               | <code>► INSERT-TEXT-HERE</code> |
| [ARB.cpp](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/ARB/ARB.cpp)           | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>ARB.examples.motorControl</summary>

| File                                                                                                                                        | Summary                         |
| ---                                                                                                                                         | ---                             |
| [motorControl.ino](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/ARB/examples/motorControl/motorControl.ino) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>ARB.examples.pushButton</summary>

| File                                                                                                                                  | Summary                         |
| ---                                                                                                                                   | ---                             |
| [pushButton.ino](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/ARB/examples/pushButton/pushButton.ino) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>ARB.examples.BLEPeripheral</summary>

| File                                                                                                                                           | Summary                         |
| ---                                                                                                                                            | ---                             |
| [BLEPeripheral.ino](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/ARB/examples/BLEPeripheral/BLEPeripheral.ino) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>ARB.examples.I2CMux</summary>

| File                                                                                                                      | Summary                         |
| ---                                                                                                                       | ---                             |
| [I2CMux.ino](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/ARB/examples/I2CMux/I2CMux.ino) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>ARB.examples.uSonic</summary>

| File                                                                                                                      | Summary                         |
| ---                                                                                                                       | ---                             |
| [uSonic.ino](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/ARB/examples/uSonic/uSonic.ino) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>ARB.examples.serialComms</summary>

| File                                                                                                                                     | Summary                         |
| ---                                                                                                                                      | ---                             |
| [serialComms.ino](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/ARB/examples/serialComms/serialComms.ino) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>Lab3-4.task4.motorControlPWM</summary>

| File                                                                                                                                                 | Summary                         |
| ---                                                                                                                                                  | ---                             |
| [motorControlPWM.ino](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab3-4/task4/motorControlPWM/motorControlPWM.ino) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>Lab3-4.task5</summary>

| File                                                                                                             | Summary                         |
| ---                                                                                                              | ---                             |
| [ARBPi.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab3-4/task5/ARBPi.h)     | <code>► INSERT-TEXT-HERE</code> |
| [ARBPi.cpp](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab3-4/task5/ARBPi.cpp) | <code>► INSERT-TEXT-HERE</code> |
| [ARBPi.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab3-4/task5/ARBPi.py)   | <code>► INSERT-TEXT-HERE</code> |
| [task5.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab3-4/task5/task5.py)   | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>Lab3-4.task5.motorControl_joystick</summary>

| File                                                                                                                                                                   | Summary                         |
| ---                                                                                                                                                                    | ---                             |
| [motorControl_joystick.ino](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab3-4/task5/motorControl_joystick/motorControl_joystick.ino) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>Lab3-4.task5.serialComms_send</summary>

| File                                                                                                                                                    | Summary                         |
| ---                                                                                                                                                     | ---                             |
| [ARBPi.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab3-4/task5/serialComms_send/ARBPi.h)                           | <code>► INSERT-TEXT-HERE</code> |
| [ARBPi.cpp](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab3-4/task5/serialComms_send/ARBPi.cpp)                       | <code>► INSERT-TEXT-HERE</code> |
| [serialComms_send.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab3-4/task5/serialComms_send/serialComms_send.py)   | <code>► INSERT-TEXT-HERE</code> |
| [ARBPi.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab3-4/task5/serialComms_send/ARBPi.py)                         | <code>► INSERT-TEXT-HERE</code> |
| [serialComms_send1.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab3-4/task5/serialComms_send/serialComms_send1.py) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>Lab3-4.task5.serialComms_receive</summary>

| File                                                                                                                                                             | Summary                         |
| ---                                                                                                                                                              | ---                             |
| [serialComms_receive.ino](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab3-4/task5/serialComms_receive/serialComms_receive.ino) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>Lab3-4.task5.serialComms_receive1</summary>

| File                                                                                                                                                                | Summary                         |
| ---                                                                                                                                                                 | ---                             |
| [serialComms_receive1.ino](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab3-4/task5/serialComms_receive1/serialComms_receive1.ino) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>Lab3-4.task5.motorControl_WASD</summary>

| File                                                                                                                                                       | Summary                         |
| ---                                                                                                                                                        | ---                             |
| [motorControl_WASD.ino](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab3-4/task5/motorControl_WASD/motorControl_WASD.ino) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>BLE Example</summary>

| File                                                                                                                  | Summary                         |
| ---                                                                                                                   | ---                             |
| [BLECentral.c](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/BLE Example/BLECentral.c) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>BLE Example.include.simpleble_c</summary>

| File                                                                                                                                      | Summary                         |
| ---                                                                                                                                       | ---                             |
| [utils.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/BLE Example/include/simpleble_c/utils.h)           | <code>► INSERT-TEXT-HERE</code> |
| [simpleble.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/BLE Example/include/simpleble_c/simpleble.h)   | <code>► INSERT-TEXT-HERE</code> |
| [adapter.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/BLE Example/include/simpleble_c/adapter.h)       | <code>► INSERT-TEXT-HERE</code> |
| [types.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/BLE Example/include/simpleble_c/types.h)           | <code>► INSERT-TEXT-HERE</code> |
| [logging.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/BLE Example/include/simpleble_c/logging.h)       | <code>► INSERT-TEXT-HERE</code> |
| [peripheral.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/BLE Example/include/simpleble_c/peripheral.h) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>BLE Example.include.simpleble</summary>

| File                                                                                                                                            | Summary                         |
| ---                                                                                                                                             | ---                             |
| [Utils.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/BLE Example/include/simpleble/Utils.h)                   | <code>► INSERT-TEXT-HERE</code> |
| [PeripheralSafe.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/BLE Example/include/simpleble/PeripheralSafe.h) | <code>► INSERT-TEXT-HERE</code> |
| [Peripheral.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/BLE Example/include/simpleble/Peripheral.h)         | <code>► INSERT-TEXT-HERE</code> |
| [SimpleBLE.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/BLE Example/include/simpleble/SimpleBLE.h)           | <code>► INSERT-TEXT-HERE</code> |
| [AdapterSafe.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/BLE Example/include/simpleble/AdapterSafe.h)       | <code>► INSERT-TEXT-HERE</code> |
| [Types.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/BLE Example/include/simpleble/Types.h)                   | <code>► INSERT-TEXT-HERE</code> |
| [Adapter.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/BLE Example/include/simpleble/Adapter.h)               | <code>► INSERT-TEXT-HERE</code> |
| [Exceptions.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/BLE Example/include/simpleble/Exceptions.h)         | <code>► INSERT-TEXT-HERE</code> |
| [Logging.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/BLE Example/include/simpleble/Logging.h)               | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>Lab2.task1</summary>

| File                                                                                                                               | Summary                         |
| ---                                                                                                                                | ---                             |
| [ARBPi.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab2/task1/ARBPi.h)                         | <code>► INSERT-TEXT-HERE</code> |
| [serialTest.cpp](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab2/task1/serialTest.cpp)           | <code>► INSERT-TEXT-HERE</code> |
| [ARBPi.cpp](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab2/task1/ARBPi.cpp)                     | <code>► INSERT-TEXT-HERE</code> |
| [task1.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab2/task1/task1.py)                       | <code>► INSERT-TEXT-HERE</code> |
| [serialComms_send.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab2/task1/serialComms_send.py) | <code>► INSERT-TEXT-HERE</code> |
| [ARBPi.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab2/task1/ARBPi.py)                       | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>Lab2.task1.I2CMux_singlebus</summary>

| File                                                                                                                                                  | Summary                         |
| ---                                                                                                                                                   | ---                             |
| [I2CMux_singlebus.ino](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab2/task1/I2CMux_singlebus/I2CMux_singlebus.ino) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>Lab2.task1.serialComms</summary>

| File                                                                                                                                   | Summary                         |
| ---                                                                                                                                    | ---                             |
| [serialComms.ino](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab2/task1/serialComms/serialComms.ino) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>Lab2.task2</summary>

| File                                                                                                           | Summary                         |
| ---                                                                                                            | ---                             |
| [ARBPi.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab2/task2/ARBPi.h)     | <code>► INSERT-TEXT-HERE</code> |
| [ARBPi.cpp](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab2/task2/ARBPi.cpp) | <code>► INSERT-TEXT-HERE</code> |
| [ARBPi.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab2/task2/ARBPi.py)   | <code>► INSERT-TEXT-HERE</code> |
| [task2.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab2/task2/task2.py)   | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>Lab2.task2.I2CMux_serial</summary>

| File                                                                                                                                         | Summary                         |
| ---                                                                                                                                          | ---                             |
| [I2CMux_serial.ino](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab2/task2/I2CMux_serial/I2CMux_serial.ino) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>Lab9</summary>

| File                                                                                                                                           | Summary                         |
| ---                                                                                                                                            | ---                             |
| [obstacle_test.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab9/obstacle_test.py)                         | <code>► INSERT-TEXT-HERE</code> |
| [ARBPi.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab9/ARBPi.h)                                           | <code>► INSERT-TEXT-HERE</code> |
| [ARBPi.cpp](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab9/ARBPi.cpp)                                       | <code>► INSERT-TEXT-HERE</code> |
| [TurtleBot.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab9/TurtleBot.py)                                 | <code>► INSERT-TEXT-HERE</code> |
| [ARBPi.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab9/ARBPi.py)                                         | <code>► INSERT-TEXT-HERE</code> |
| [speed_conversion_new_test.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab9/speed_conversion_new_test.py) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>Lab6</summary>

| File                                                                                                                       | Summary                         |
| ---                                                                                                                        | ---                             |
| [mask.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab6/mask.py)                       | <code>► INSERT-TEXT-HERE</code> |
| [blob.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab6/blob.py)                       | <code>► INSERT-TEXT-HERE</code> |
| [TurtleBot.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab6/TurtleBot.py)             | <code>► INSERT-TEXT-HERE</code> |
| [test_camera_fps.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab6/test_camera_fps.py) | <code>► INSERT-TEXT-HERE</code> |
| [aruco_test.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab6/aruco_test.py)           | <code>► INSERT-TEXT-HERE</code> |
| [test_camera.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab6/test_camera.py)         | <code>► INSERT-TEXT-HERE</code> |
| [blobhd.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab6/blobhd.py)                   | <code>► INSERT-TEXT-HERE</code> |
| [gen_aruco_pdf.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab6/gen_aruco_pdf.py)     | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>UCAS_showcase.generalControl</summary>

| File                                                                                                                                               | Summary                         |
| ---                                                                                                                                                | ---                             |
| [Infrared.cpp](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/UCAS_showcase/generalControl/Infrared.cpp)             | <code>► INSERT-TEXT-HERE</code> |
| [Infrared.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/UCAS_showcase/generalControl/Infrared.h)                 | <code>► INSERT-TEXT-HERE</code> |
| [Motors.cpp](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/UCAS_showcase/generalControl/Motors.cpp)                 | <code>► INSERT-TEXT-HERE</code> |
| [Ultrasonics.cpp](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/UCAS_showcase/generalControl/Ultrasonics.cpp)       | <code>► INSERT-TEXT-HERE</code> |
| [Joystick.cpp](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/UCAS_showcase/generalControl/Joystick.cpp)             | <code>► INSERT-TEXT-HERE</code> |
| [generalControl.ino](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/UCAS_showcase/generalControl/generalControl.ino) | <code>► INSERT-TEXT-HERE</code> |
| [Joystick.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/UCAS_showcase/generalControl/Joystick.h)                 | <code>► INSERT-TEXT-HERE</code> |
| [Motors.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/UCAS_showcase/generalControl/Motors.h)                     | <code>► INSERT-TEXT-HERE</code> |
| [Ultrasonics.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/UCAS_showcase/generalControl/Ultrasonics.h)           | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>UCAS_showcase.initial_testing</summary>

| File                                                                                                                                                                                    | Summary                         |
| ---                                                                                                                                                                                     | ---                             |
| [finite_state_machine_example.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/UCAS_showcase/initial_testing/finite_state_machine_example.py)           | <code>► INSERT-TEXT-HERE</code> |
| [shape_detection.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/UCAS_showcase/initial_testing/shape_detection.py)                                     | <code>► INSERT-TEXT-HERE</code> |
| [WASD_control_obstacle_avoidance.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/UCAS_showcase/initial_testing/WASD_control_obstacle_avoidance.py)     | <code>► INSERT-TEXT-HERE</code> |
| [WASD_control.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/UCAS_showcase/initial_testing/WASD_control.py)                                           | <code>► INSERT-TEXT-HERE</code> |
| [TurtleBot.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/UCAS_showcase/initial_testing/TurtleBot.py)                                                 | <code>► INSERT-TEXT-HERE</code> |
| [shape_detect_test.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/UCAS_showcase/initial_testing/shape_detect_test.py)                                 | <code>► INSERT-TEXT-HERE</code> |
| [wall_follower_with_camera_2.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/UCAS_showcase/initial_testing/wall_follower_with_camera_2.py)             | <code>► INSERT-TEXT-HERE</code> |
| [showcase.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/UCAS_showcase/initial_testing/showcase.py)                                                   | <code>► INSERT-TEXT-HERE</code> |
| [wall_follower_with_camera_3_slow.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/UCAS_showcase/initial_testing/wall_follower_with_camera_3_slow.py)   | <code>► INSERT-TEXT-HERE</code> |
| [WASD_control_obstacle_avoidance_2.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/UCAS_showcase/initial_testing/WASD_control_obstacle_avoidance_2.py) | <code>► INSERT-TEXT-HERE</code> |
| [wall_follower_fsm.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/UCAS_showcase/initial_testing/wall_follower_fsm.py)                                 | <code>► INSERT-TEXT-HERE</code> |
| [shape_detect_test_2.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/UCAS_showcase/initial_testing/shape_detect_test_2.py)                             | <code>► INSERT-TEXT-HERE</code> |
| [aruco_detect_test.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/UCAS_showcase/initial_testing/aruco_detect_test.py)                                 | <code>► INSERT-TEXT-HERE</code> |
| [wall_follower_with_camera.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/UCAS_showcase/initial_testing/wall_follower_with_camera.py)                 | <code>► INSERT-TEXT-HERE</code> |
| [wall_follower_with_camera_3.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/UCAS_showcase/initial_testing/wall_follower_with_camera_3.py)             | <code>► INSERT-TEXT-HERE</code> |
| [threshold_inRange.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/UCAS_showcase/initial_testing/threshold_inRange.py)                                 | <code>► INSERT-TEXT-HERE</code> |
| [colour_detect_test.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/UCAS_showcase/initial_testing/colour_detect_test.py)                               | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>Lab1</summary>

| File                                                                                                                     | Summary                         |
| ---                                                                                                                      | ---                             |
| [pigpio_example.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab1/pigpio_example.py) | <code>► INSERT-TEXT-HERE</code> |
| [pigpio_example.c](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab1/pigpio_example.c)   | <code>► INSERT-TEXT-HERE</code> |
| [task6.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab1/task6.py)                   | <code>► INSERT-TEXT-HERE</code> |
| [task5.c](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab1/task5.c)                     | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>Lab8</summary>

| File                                                                                                                         | Summary                         |
| ---                                                                                                                          | ---                             |
| [ARBPi.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab8/ARBPi.h)                         | <code>► INSERT-TEXT-HERE</code> |
| [task2eight.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab8/task2eight.py)             | <code>► INSERT-TEXT-HERE</code> |
| [ARBPi.cpp](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab8/ARBPi.cpp)                     | <code>► INSERT-TEXT-HERE</code> |
| [task2rectangle.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab8/task2rectangle.py)     | <code>► INSERT-TEXT-HERE</code> |
| [task3.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab8/task3.py)                       | <code>► INSERT-TEXT-HERE</code> |
| [joystickControl.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab8/joystickControl.py)   | <code>► INSERT-TEXT-HERE</code> |
| [TurtleBot.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab8/TurtleBot.py)               | <code>► INSERT-TEXT-HERE</code> |
| [ARBPi.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab8/ARBPi.py)                       | <code>► INSERT-TEXT-HERE</code> |
| [task2square.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab8/task2square.py)           | <code>► INSERT-TEXT-HERE</code> |
| [task2line.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab8/task2line.py)               | <code>► INSERT-TEXT-HERE</code> |
| [infraredSensor.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab8/infraredSensor.py)     | <code>► INSERT-TEXT-HERE</code> |
| [task2circle.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab8/task2circle.py)           | <code>► INSERT-TEXT-HERE</code> |
| [ultrasoundSensor.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab8/ultrasoundSensor.py) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>Lab8.generalControl</summary>

| File                                                                                                                                      | Summary                         |
| ---                                                                                                                                       | ---                             |
| [generalControl.ino](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Lab8/generalControl/generalControl.ino) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>Showcase</summary>

| File                                                                                                               | Summary                         |
| ---                                                                                                                | ---                             |
| [ARBPi.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Showcase/ARBPi.h)           | <code>► INSERT-TEXT-HERE</code> |
| [ARBPi.cpp](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Showcase/ARBPi.cpp)       | <code>► INSERT-TEXT-HERE</code> |
| [TurtleBot.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Showcase/TurtleBot.py) | <code>► INSERT-TEXT-HERE</code> |
| [ARBPi.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Showcase/ARBPi.py)         | <code>► INSERT-TEXT-HERE</code> |
| [showcase.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/Showcase/showcase.py)   | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>ARBPi library for Pi Serial comms</summary>

| File                                                                                                                                            | Summary                         |
| ---                                                                                                                                             | ---                             |
| [serialTest.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/ARBPi library for Pi Serial comms/serialTest.py)   | <code>► INSERT-TEXT-HERE</code> |
| [ARBPi.h](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/ARBPi library for Pi Serial comms/ARBPi.h)               | <code>► INSERT-TEXT-HERE</code> |
| [serialTest.cpp](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/ARBPi library for Pi Serial comms/serialTest.cpp) | <code>► INSERT-TEXT-HERE</code> |
| [ARBPi.cpp](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/ARBPi library for Pi Serial comms/ARBPi.cpp)           | <code>► INSERT-TEXT-HERE</code> |
| [ARBPi.py](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/master/ARBPi library for Pi Serial comms/ARBPi.py)             | <code>► INSERT-TEXT-HERE</code> |

</details>

</details>

---

##  Getting Started

**Software and Libraries**

To run this project, you will need:

For the Arduino:

* Arduino IDE
* C++ Standard Library
* ARB library (included, explained below)

On the Raspberry Pi:

* Python 3.6 or above
* Python Packages: `numpy`, `picamera`, `picamera.array`, `cv2`, `aruco`, `matplotlib`, `ctypes`, `csv`, `os`, `threading`

On your computer:

* PuTTY: To establish comms with the Pi using *SSH*. (or equivalent)
* WinSCP: For FTP comms and file transfer. (or equivalent)
* XMing: To forward camera image data. (or equivalent)

**Hardware**

The Raspberry Pi Robotics Kit (RPRK) is designed to help students build and program a mobile robot capable of solving a maze through localization and mapping. The robot uses various sensors, including wheel encoders, a front-facing camera, two side ultrasonic sensors, and a front infrared sensor, to navigate its environment. The kit includes:

* **Arduino Robotics Board (ARB)**: A custom board based on the *Arduino Nano 33 BLE*, featuring motor drivers, connectors for IR distance sensors, and headers for ultrasonic sensors.
* **Arduino Nano 33 BLE**: Integrated into the **ARB**.
* **Raspberry Pi Model 4**: Available with varying RAM from 2GB to 8GB, with 2GB being common and sufficient for the project.
* **USB to UART HAT**: Installed on the Raspberry Pi for serial console access.
* **MicroSD card**: Preloaded with a custom *Raspbian Buster* image containing necessary software.
* **Raspberry Pi Camera 3**: Comes with a ribbon cable.
* **USB battery bank** and **Micro USB and USB-C cables**.
* **Motors and Wheels**: Includes two 298:1 micro metal gearmotors with encoders, mounted in 3D-printed red mounts, and two narrow wheels.
* **Pololu 3/4" ball caster**: Includes mounting hardware, noting that the screws are imperial and not interchangeable with metric screws.
* **Sensor and Motor Driver kit**: Contains an IR sensor, a motor driver, and two ultrasonic range sensors.
* **Assorted 3D printed and laser cut parts**: Includes mounts for sensors and chassis components.
* **Assorted Velcro strips and fasteners**: For mounting and assembly.

###  Installation

1. **Clone the repository** to your local machine or **download the entire project directory**.

   <h4>From <code>source</code></h4>

   > 1. Clone the rprk_turtlebot_lab_sessions repository:
   >
   > ```console
   > $ git clone https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git
   > ```
   >
   > 2. Change to the project directory:
   > ```console
   > $ cd rprk_turtlebot_lab_sessions
   > ```

2. **Install the ARB Library** on your Arduino:
   1. In the *Arduino IDE*, go to the menu bar and select **Sketch** > **Include Library** > **Add .ZIP Library....**
   2. Zip the ARB directory and navigate to where you have saved your **"ARB.zip"** file.
   3. Select the file and click on **'Open'**. The IDE will then install the library.
   4. Verify Installation:
      - To check if the library has been successfully installed, go back to **Sketch** > **Include Library**. You should see the library named "ARB" at the bottom of the drop-down menu.
      - Click on it to include the library in your current sketch, which should automatically insert an include statement like `#include <ARB.h>` at the top of your sketch.

3. **Open a connection to the Raspberry Pi** using PuTTY or directly through HDMI. (Windows instructions). Further instructions in [Lab 1](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions/blob/main/Lab1/autumn_lab_1.pdf):
   1. Connect to the Raspberry Pi with your laptop using the serial **USB to UART HAT** and a USB cable.
   2. Check what *COM* port the device is connected to using "Device Manager"
   3. Establish a `Serial` connection with PuTTY using the device *COM* port and baud rate 115200.
   
   Alternatively, you can connect a screen and keyboard directly to the Raspberry Pi to access the terminal directly.

   4. Login using your login details for the Raspberry Pi in CLI. For the lab RPRK devices, the details are the following:
      * **SSID:** *pi*
      * **Password:** *raspberry*

4. **Connect the Pi to a local WiFi** network and **check the device's IP address** on the network.
   1. Type `sudo raspi-config` in the command line to open the configuration screen.
   2. Go to **“2: Network Options”** and then **“N2 Wireless LAN”** and enter the SSID and passphrase for your network.
   3. Go to "Finish" and wait a few moments for the Raspberry Pi to connect.
   4. Type `ifconfig` on the terminal.
   5. Look for the section called *wlan0*. You should see your IP address there (e.g 144.32.70.210).
   6. Take note of your IP, it can be used to connect to the board through SSH or to transfer files with FTP. You can now close the serial PuTTY or direct connection.

5. **Connect through SSH using PuTTY** (Windows instructions)
   1. Open PuTTY again.
   2. Establish an `SSH` connection using host name *"username@ip_address"* (e.g pi@144.32.70.210) and port 22, using the previously established IP address.
   3. To forward camera image data from the Pi to your computer, you must:
      * Have [XMing](http://www.straightrunning.com/XmingNotes/) installed in your device. Further instructions in [Lab 6](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions/blob/main/Lab6/autumn_lab_6.pdf)
      * Execute XMing before establishing a connection. It will run in the background.
      * In PuTTY, go to **Connections** > **SSH** > **X11** and check the box that says *'Enable X-11 forwarding'*.
   4. If you wish, save the session under your preferred name by going to **Session** > **"Load, save or delete a stored session"**. Write your session name under *Saved Sessions* and click **"Save"**.
   5. Click **"Open"** at the bottom-right of the window.
   6. Login using your login details for the Raspberry Pi in CLI. For the lab RPRK devices, the details are the following:
      * **SSID:** *pi*
      * **Password:** *raspberry*

6. **View, add and modify files using WinSCP** (Windows instructions). Further instructions in [Lab 1](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions/blob/main/Lab1/autumn_lab_1.pdf)
   1. Open WinSCP
   2. Create a "New Site" with the following details:
      * **File Protocol**: *SFTP*
      * **Host Name**: The device's IP address for the network in format *XXX.XX.XX.XXX* (e.g *144.32.70.210*). Refer to step 4 in [Installation](#installation).
      * **Port**: *22*
      * **User Name**: Your SSID for the Raspberry Pi. In lab devices: ***pi***.
      * **Password**: Your password for the Raspberry Pi. In lab devices: ***raspberry***.
   3. Click "Login". After a connection is established you should be able to see the files in the Pi.

7. **Upload all project files** to the Raspberry Pi using WinSCP.

8. **Install Python dependencies** with the CLI in SSH PuTTY:

   All of the Python libraries can be installed using pip:

   ```bash
   pip install numpy opencv-python picamera matplotlib ...
   ```

###  Usage

**Running the Code**

To run Arduino files, upload the respective `.ino` files to the Arduino Nano using the Arduino IDE.

For the Raspberry Pi, navigate to the specific lab or showcase directory and run the Python scripts through the terminal:

```bash
python3 <script_name>.py
```

Ensure that the `ARBPi` file and the `Turtlebot` files or other dependencies are located in the same folder when running a script that requires their functions.

Ensure that the Raspberry Pi and ARB are connected via UART, as the scripts and sketches often communicate over this channel.

---

##  ARB and ARBPi

The ARB (Arduino Robotics Board) and ARBPi libraries are crucial for communication between the Arduino and Raspberry Pi. They handle low-level operations like reading and writing to registers that control motors, read sensors, and manage other peripherals.

### ARB library

The ARB (Arduino Robotics Board) library runs in the Arduino Nano and is designed to simplify the interaction between the hardware components on the ARB and the software controlling it, in conjunction with a Raspberry Pi. This library is integral for controlling the motors, sensors, and serial communication in your robotics projects.

**Key Components of the ARB Library**

*Header File (`ARB.h`)*:

* Defines all the necessary pins used by the ARB with easy-to-understand names such as **MOTOR_DIRA** for motor direction control and **USONIC1** to **USONIC4** for ultrasonic sensors.
* It includes standard Arduino headers and defines constants for the **I2C multiplexer address** and various **GPIO** pins.
* Declares an external array `reg_array` of 128 bytes to manage data communication between the Raspberry Pi and the Arduino.
* Provides function prototypes for initialization (`ARBSetup`), register manipulation (`getRegister`, `putRegister`), and I2C bus management (`setI2CBus`).

*Source File (`ARB.cpp`)*:

* Implements the functions declared in the header. The `ARBSetup` function initializes **I2C** communication and optionally the serial communication depending on the passed parameter.
* The `getRegister` and `putRegister` functions manage data in the `reg_array`, facilitating communication between the Arduino and any connected device like the Raspberry Pi.
* The `setI2CBus` function controls which bus on the **I2C multiplexer** is active, allowing the selection of different sensor sets connected to the Arduino.
* Provides a utility function `uSecToCM` to convert time (in microseconds) to distance (in centimeters), which is useful for ultrasonic distance measurements.

**`getRegister` and `putRegister` Functions in the ARB Library**

The `getRegister` and `putRegister` functions are crucial components of the ARB library, allowing for efficient data communication between the Arduino and other devices, such as the Raspberry Pi. These functions manage data within an array named `reg_array`, which acts as a collection of registers used to store and retrieve data dynamically during runtime.

*`getRegister` Function*

The `getRegister` function is designed to access data from the `reg_array`. Here's how it works:

* **Prototype**: char getRegister(int reg);
* **Parameters**: It takes a single integer reg, which represents the index of the register in the * reg_array from which data is to be retrieved.
* **Returns**: The function returns a char value, which is the data stored at the specified register index.

   **Code Snippet**:

   ```cpp
   // C++
   char getRegister(int reg){
      return reg_array[reg];
   }
   ```

   **Usage**:

   This function is typically used when you need to read a value from a specific register that may have been written to by another part of your program or from an external device like the Raspberry Pi. For instance, you might store sensor data or configuration settings in these registers.

*`putRegister` Function*

The `putRegister` function allows writing data to a specific register in the `reg_array`. Here’s a detailed look:

* **Prototype**: void putRegister(int reg, char data);
* **Parameters**:
   * `int reg`: The index of the register where the data will be stored.
   * `char data`: The data to be stored in the register.

   **Code Snippet**:

   ```cpp
   // C++
   void putRegister(int reg, char data){
      reg_array[reg] = data;
   }
   ```

   **Usage**:

   This function is crucial for updating the contents of a register, which could influence the behavior of the robot or other parts of the system. For example, it could be used to update control parameters, set desired motor speeds, or store temporary data needed for computations.

**Example Usage**

Here’s a basic example of how the ARB library functions might be used in an Arduino sketch:

```cpp
// C++
#include <ARB.h>

void setup() {
    ARBSetup(true); // Initialize with serial communication enabled
}

void loop() {
    // Example of setting a register value
    putRegister(0, 120); // Put 120 in register 0

    // Example of reading a register value
    char val = getRegister(0);

    // Use the infrared sensor connected to I2C bus 1
    setI2CBus(1);
}
```

Here is how you might use both getRegister and putRegister in a practical scenario:

```cpp
// C++
void setup() {
    ARBSetup(); // Initialize ARB without serial communication
}

void loop() {
    // Setting a register value to store a motor speed setting
    putRegister(10, 50);  // Assume register 10 is designated for motor speed

    // Later in the loop, or in another function, you retrieve this motor speed
    char motorSpeed = getRegister(10);

    // Use the motor speed to control a motor
    analogWrite(MOTOR_PWMA, motorSpeed);  // Assuming MOTOR_PWMA controls a motor's speed
}
```

A demo usage of the ARB library for serial communication can also be found in `examples/serialComms.ino`.

**Overview of Example Files in the ARB Examples Directory**

Each example in the ARB library's **"examples"** directory demonstrates specific functionalities of the Arduino Robotics Board. Here is a brief overview of what each example illustrates:

1. BLEPeripheral (`BLEPeripheral.ino`)
This example showcases how to set up and use a Bluetooth Low Energy (BLE) peripheral with the Arduino. It is essential for projects that require wireless data transmission or remote control via BLE.

2. I2CMux (`I2CMux.ino`)
Demonstrates the use of an I2C multiplexer with the ARB. This is critical for projects where multiple I2C devices must share the same I2C bus without addressing conflicts.

3. motorControl (`motorControl.ino`)
Provides a basic example of how to control motors using the ARB. It includes setting up the motor drivers and controlling the speed and direction of DC motors, which is fundamental for any mobile robotics project.

4. pushButton (`pushButton.ino`)
Shows how to read the state of push buttons using the ARB. It is useful for projects that require user input or a simple interface for triggering actions.

5. serialComms (`serialComms.ino`)
This example is about setting up and using serial communication between the ARB and another device, like a Raspberry Pi or a computer. It covers sending and receiving data over serial, which is vital for debugging and complex communications.

6. uSonic (`uSonic.ino`)
Focuses on using ultrasonic sensors with the ARB to measure distances. This is particularly useful in robotics for obstacle avoidance, navigation, and environment mapping.

**Detailed Explanation of Key Examples**

Let's dive deeper into two specific examples: `motorControl` and `uSonic`.

*`motorControl.ino`*

This script initializes and controls two DC motors connected to the ARB. It handles setting the direction and speed of each motor through PWM signals, which are essential for driving the motors in forward or reverse directions.

Key Snippets:

```cpp
// C++
void setup() {
    pinMode(MOTOR_PWMA, OUTPUT);  // Set motor A PWM pin as output
    pinMode(MOTOR_DIRA, OUTPUT);  // Set motor A direction pin as output
}

void loop() {
    analogWrite(MOTOR_PWMA, 128);  // Set speed for motor A
    digitalWrite(MOTOR_DIRA, HIGH); // Set direction for motor A
    delay(1000);                    // Run for 1 second
    digitalWrite(MOTOR_DIRA, LOW);  // Change direction
    delay(1000);                    // Run in the opposite direction for 1 second
}
```

*`uSonic.ino`*

This script demonstrates how to use an ultrasonic sensor connected to the ARB to measure distances. The script calculates the distance by timing how long it takes for an ultrasonic pulse to return to the sensor.

Key Snippets:

```cpp
// C++
void setup() {
    pinMode(USONIC1, INPUT);  // Set the ultrasonic sensor pin as input
}

void loop() {
    long duration, distance;
    digitalWrite(USONIC1, LOW);
    delayMicroseconds(2);
    digitalWrite(USONIC1, HIGH);
    delayMicroseconds(10);
    digitalWrite(USONIC1, LOW);
    duration = pulseIn(USONIC1, HIGH);
    distance = duration / 29 / 2;  // Calculate distance
    Serial.print("Distance: ");
    Serial.println(distance);
    delay(1000);
}
```

### ARBPi library

**Purpose and Functionality:**

The ARBPi library runs in the Raspberry Pi and provides serial communication between the Raspberry Pi and the Arduino boards. The library offers a dual-interface, supporting both C++ and Python, thereby accommodating a wide range of programming preferences and project requirements.

**Technical Stack and Integration:**

**C++ Components**: Core functionalities are implemented in C++, ensuring high performance and direct access to low-level system resources.

**Python Interface**: Python bindings are provided to leverage the ease of scripting and rapid development capabilities of Python, making it ideal for higher-level applications and quick testing.

**Key Components:**

*C++ Source Files (`ARBPi.cpp` and `ARBPi.h`)*:
   - `ARBPi.cpp`: Contains the implementation of serial communication functions such as setting up the serial port, reading and writing to registers on the **Arduino**.
   - `ARBPi.h`: Header file that declares the functions and constants used by `ARBPi.cpp`.

*Python Module (`ARBPi.py`)*:
   Wraps the **C++** library using **Python’s** ctypes module, providing Pythonic access to the underlying serial communication functions.

*Compiled Library (`libARBPi.so`)*:
   A shared library compiled from the **C++** code, enabling dynamic linking from Python or other **C++** programs.

*Test Files (`serialtest`, `serialTest.cpp`, and `serialTest.py`)*:
   Executable and scripts for testing the functionality of the library to ensure proper operation of serial communications.

*Libraries and Dependencies*:
   - `wiringPi`: Used in the **C++** code for handling **GPIO** and serial communications on the Raspberry Pi.
   - `ctypes`: Utilized in the **Python** script to interface with the **C++** shared library.

**Setup and Usage Instructions:**

*Compiling C++ Code:*

To compile the C++ part of the library, you would use g++ with appropriate flags to link against necessary libraries, such as wiringPi:

```bash
g++ -o ARBPi ARBPi.cpp -lwiringPi
```

*Running Python Scripts:*

Ensure Python is installed along with ctypes. The Python script can be run by importing it to your code as follows:

```python
# Python
from ARBPi import *
```

**Initialization Process:**

C++ and Python interfaces include an initialization function to set up the serial connection:
```cpp
// C++
void ARBPiSetup() {
    serialDevice = serialOpen(SERIAL, 115200);
}
```
```python
# Python
def ARBPiSetup(serialPath="/dev/ttyUSB0"):
    _ARBPi.ARBPiSetup(ctypes.c_char_p(serialPath.encode('ascii')))
```

Reading and Writing Registers:

To read a register:
```cpp
// C++
char getRegister(int reg) {
    serialPutchar(serialDevice, reg);
    while(serialDataAvail(serialDevice) < 1) {}
    return serialGetchar(serialDevice);
}
```
```python
# Python
def getRegister(reg):
    return int(_ARBPi.getRegister(ctypes.c_int(reg)))
```

To write to a register:
```cpp
// C++
void putRegister(int reg, char data) {
    serialPutchar(serialDevice, reg + 128);
    serialPutchar(serialDevice, data);
}
```
```python
# Python
def putRegister(reg, data):
    _ARBPi.putRegister(ctypes.c_int(reg), ctypes.c_byte(data))
```

These snippets illustrate the direct interaction with hardware through serial interfaces, encapsulating complex operations into simple, reusable API calls.

---

## Lab Sessions

### Lab 1: Robot Kit Assembly and RPi Programming

Lab Session 1 focuses on two primary areas: assembling the basic robot chassis from the York Robotics Kit and introducing students to programming on the Raspberry Pi in both C++ and Python. Here’s a detailed breakdown of how the lab is structured and how the provided code files relate to the tasks.

**Aims and Objectives:**

* **Assembly of the robot chassis**: Students begin by assembling the mechanical parts provided in the kit, which serves as the physical platform for all subsequent labs.
* **Programming introduction**: The lab introduces basic programming tasks using the Raspberry Pi, focusing on using GPIO pins for tasks like blinking an LED.

**Hardware and Software Setup:**

* The kit includes various mechanical parts (motors, wheels, sensors), a Raspberry Pi 4, camera, and other electronic components.
* Students use the Raspberry Pi OS for programming tasks.

**Lab Tasks and Associated Code Files:**

*Task 1: Assemble the Chassis of Your Robot*

* **Objective**: Using a detailed instructional document, students assemble the mechanical structure of their robot. This task is foundational and does not involve the code files directly.

*Task 2: Wire Together Serial Communications and an LED Light*

* **Objective**: Learn to handle simple GPIO output (blinking an LED) and setup basic serial communications

*Task 3: Communicate with the Raspberry Pi*

* **Objective**: Establish a connection to the Raspberry Pi via a serial connection for command-line interaction, crucial for debugging and further development.

*Task 5: Write a C Program to Blink the LED Using pigpio*

* **Objective**: Task 5 directs students to write a C program that controls an LED's on/off state via GPIO using the pigpio library on the Raspberry Pi. This task is about practicing low-level GPIO access, mirroring typical microcontroller programming but on a more sophisticated platform.

* **Code Implementation**:

   `pigpio_example.c` and `task5.c` provided in the lab files are the direct application of what Task 5 asks for. Here's an explanation of how the code works:

   ```c
   // C
   #include <pigpio.h>
   #define LED 5

   int main() {
      if (gpioInitialise() < 0) {
         printf("Initialization failed\n");
         return 1;
      }
      gpioSetMode(LED, PI_OUTPUT);
      for(int i = 0; i < 10; i++) {
         gpioWrite(LED, 1); // Turn LED on
         time_sleep(0.5);  // Keep it on for 0.5 seconds
         gpioWrite(LED, 0); // Turn LED off
         time_sleep(0.5);  // Keep it off for 0.5 seconds
      }
      gpioTerminate(); // Free resources
   }
   ```

   **Execution**: Students are instructed to compile and run the program, potentially modifying parameters like blink rate or the number of blinks to see real-time effects on the hardware.

*Task 6: Blink the LED Using Python and the pigpiod Daemon*

* **Objective**: Task 6 extends the concepts from Task 5 into Python, using the pigpiod daemon for GPIO control. This approach offers the advantage of not requiring sudo permissions, emphasizing security and simplicity in managing GPIO pins.

* **Code Implementation**:

   `pigpio_example.py` and `task6.py` demonstrates how to achieve similar functionality as Task 5 but in Python. This script also serves as a direct solution to Task 6:

   ```python
   #!/usr/bin/python3
   import pigpio
   import time

   LED = 5
   pi = pigpio.pi()  # Connect to the pigpiod daemon

   if not pi.connected:
      exit()

   pi.set_mode(LED, pigpio.OUTPUT)

   try:
      for i in range(10):
         pi.write(LED, 1)  # Turn LED on
         time.sleep(0.5)   # Keep it on for 0.5 seconds
         pi.write(LED, 0)  # Turn LED off
         time.sleep(0.5)   # Keep it off for 0.5 seconds
   finally:
      pi.stop()  # Disconnect from the pigpiod daemon
   ```

   **Execution**: The script is executed using Python3 without needing root privileges. Students are encouraged to modify the blink rate and experiment with continuous execution using loops, akin to how they would control other robotic behaviors programmatically.

### Lab 2: Embedded Navigational Sensing

**Aims and Objectives**

Lab 2 builds on the foundational skills developed in Lab 1, focusing on interfacing additional sensors for navigational tasks. The main goals are to add a display and distance measurement sensors to the Raspberry Pi, learn to read sensor data, display it, and communicate these readings to other devices.

**Files and Tasks in Lab 2**

Lab 2 contains several tasks and associated files organized into two main directories: `task1` and `task2`.

*Task 1: Connect and Test Communication*

* **Objective**: Establish a communication link between the Raspberry Pi and an Arduino configured to simulate sensor readings. This simulates reading data from a sensor over I2C by first interacting with the Arduino.

* **Key Files**:
   `task1.py`: This Python script demonstrates how to open a serial channel to the Arduino, read several registers to simulate receiving sensor data, and write data to a register to simulate sending commands or configurations.

   * **Code Explanation (`task1.py`)**:

   ```python
   # Python
   from ARBPi import *

   # Setup the ARB functions
   ARBPiSetup(SERIAL)

   # Read the first 5 registers and print their contents
   print("Read Test, reading registers 0-9")
   for i in range(11):
      print(f"Register {i}: {getRegister(i)}")

   # Write to the 30th register, then read it back to verify
   print("Write Test, reading register 30")
   original_value = getRegister(30)
   print(f"Original value in register 30: {original_value}")
   print("Writing 48 to register 30")
   putRegister(30, 48)
   new_value = getRegister(30)
   print(f"New value in register 30: {new_value}")
   ```

   This script sets up serial communication, reads multiple registers, modifies a register, and reads it back to verify the write operation, demonstrating basic I2C communication skills.

*Task 2: Sensor Data Acquisition*

* **Objective**: Extend the communication to handle specific sensor data, simulating an IR sensor read. The task involves fetching distance measurements, converting them into a usable format, and processing/displaying this information.

* **Key Files**:
   `task2.py`: Focuses on reading a distance value from a simulated IR sensor connected via the Arduino setup and displaying this value.

   * **Code Explanation (`task2.py`)**:

   ```python
   # Python
   from ARBPi import *
   import time

   def read_IR_sensor(register=0):
      # Simulate reading a distance from the IR sensor
      return getRegister(register)  # Assuming distance comes as an integer in cm

   def main():
      ARBPiSetup(SERIAL)  # Setup serial communication
      print("System initialized, reading IR sensor data...")
      
      while True:
         distance = read_IR_sensor()
         print(f"The distance read from the IR sensor is {distance} cm")
         time.sleep(0.2)  # Polling interval

   if __name__ == '__main__':
      try:
         main()
      except KeyboardInterrupt:
         print('Interrupted by user')
   ```

   This script continuously reads from a simulated IR sensor over I2C and prints the distance. It demonstrates handling real-time data from sensors, essential for tasks like obstacle avoidance in robotics.

### Lab 3-4 Overview: PWM and Motor Control

Lab Sessions 3-4 focus on advanced motor control techniques using Pulse Width Modulation (PWM) and interfacing with H-bridge circuits to control motors using the Raspberry Pi. The sessions are structured to guide students through the practical aspects of motor control in robotics, including driving motors in different directions, controlling speed, and implementing braking.

**Main Concepts Covered**:

* **H-Bridge Motor Control**: Understanding the operation of H-bridge circuits to control motor direction.
* **PWM**: Using PWM for speed control of motors.
* **Microcontroller Interface**: Using Raspberry Pi GPIO for motor control.

**Tasks and Files in Lab 3-4**

The lab is divided into several tasks, structured to sequentially build the students' skills in motor control:

*Task 4: PWM and Basic Motor Control*

* **Objective**: Implement motor control using an H-bridge and PWM signals.
* **Key Files**:
   `motorControlPWM.ino`: Arduino sketch demonstrating how to control motor speed and direction using PWM.

   This file contains Arduino code that demonstrates how to interface with the TB6612FNG motor driver using PWM signals. The code involves initializing the motor driver pins and then using PWM outputs to control motor speed and direction.

*Task 5: Advanced Motor Control and User Interaction*

* **Objective**: Enhance motor control with user input to drive the robot using keyboard commands.
* **Key Files**:
   `task5.py`: Python script for controlling robot motors based on keyboard input using the **ARBPi** library for serial communication.

   This Python script enhances the interactivity by allowing users to control motor actions through keyboard inputs. It uses the previously set up serial communication to send commands to the Arduino, which then controls the motors accordingly. Must be run in the Raspberry Pi while `motorControl_WASD.ino` is running on the Arduino.

### Lab 5 Overview: Wheel Odometry

Lab Session 5 is centered around the integration and utilization of wheel encoders to enhance the movement control of a robot. This lab introduces the students to the practical aspects of using encoder feedback to precisely control the distance and speed of a robot's motion.

**Learning Outcomes**:

* Understanding of how wheel encoders function and their application in robotics.
* Measurement of distance and control of robot motion using GPIO pin feedback from the encoders.
* Execution of controlled movement patterns based on feedback from wheel encoders.

**Key Tasks and Associated Files**

The lab is structured around several key tasks that guide students through the process of integrating and programming wheel encoders:

*Task 1: Wire up the Wheel Encoders to the Raspberry Pi*

Students are expected to connect the outputs from a quadrature encoder to the Raspberry Pi, using the knowledge they've gained about encoder operation and interfacing.

* **Key File(s)**: `encoderReading.ino` and `encoderSpeed.ino`: Includes Arduino code for measuring and calculating the speed based on encoder counts.

*Task 2: Read in the Encoder Counts*

Building upon the initial setup, this task involves programming the Raspberry Pi to read encoder signals and calculate movement parameters such as speed and distance.

* **Key File(s)**:
   * `encoderSerial.ino`: : Contains Arduino code to read encoder outputs and send them to the Raspberry Pi.
   * `task2.py` and `task2p1.py`: Provides a Python script that includes functionality for controlling the robot's movement based on user control and encoder feedback, reading encoder data through the serial regsiters using the **ARB** library. Must be run at the same time as `encoderMotorWASD.ino` and `encoderSerial.ino`.

*Task 3: Motion Estimation Based on Odometry*

This task requires students to estimate the robot's speed and distance covered using the encoder data. This involves programming to handle asynchronous data collection and real-time computation.

* **Code Explanation**: `task3.py`:

   ```python
   # Python
   from ARBPi import *
   from TurtleBot import TurtleBot

   import time
   import math

   def main():
      print("Setting up ARB")
      ARBPiSetup(SERIAL)  # Initialize serial communication
      tb = TurtleBot()
      
      # Move forward
      tb.motors.move(20)
      time.sleep(1)
      
      # Rotate right
      tb.motors.rotate(math.pi/2)
      time.sleep(1)
      
      # Rotate left
      tb.motors.rotate(-math.pi/2)
      time.sleep(1)
      
      # Move backward
      tb.motors.move(-20)

   if __name__ == '__main__':
      try:
         main()
      except KeyboardInterrupt:
         print('Interrupted!')
   ```

   This script demonstrates controlling the TurtleBot class, which abstracts motor operations, including moving forward, rotating, and handling encoder feedback.

### Lab 6 Overview: Robot Vision with OpenCV

Lab 6 focuses on integrating computer vision capabilities into a robotics platform using OpenCV and a Raspberry Pi camera module. This session is designed to familiarize students with the basics of image processing in a practical robotics context.

**Aims and Objectives**:

* **Objective**: To explore and implement various computer vision techniques using the OpenCV library to enhance the functionality of a robotic system.

* **Learning Outcomes**:
   * Operate and manipulate the Raspberry Pi camera module.
   * Execute image processing tasks such as color detection, blob detection, and marker detection.
   * Develop and run OpenCV Python code.

**Key Files and Their Functions**

1. `test_camera.py`

   This script initializes the Raspberry Pi camera and captures video frames continuously. It uses OpenCV to display these frames in real time, allowing students to observe the camera's output directly on their screens.

   *Code Overview*:

   ```python
   # Python
   import picamera
   import picamera.array
   import time
   import cv2

   camera = picamera.PiCamera()
   camera.rotation = 180
   camera.resolution = (640, 480)
   rawCapture = picamera.array.PiRGBArray(camera, size=camera.resolution)

   for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
      image = frame.array
      cv2.imshow("Frame", image)
      rawCapture.truncate(0)
      if cv2.waitKey(1) & 0xFF == ord('q'):
         break
   ```

   This script captures frames and displays them using `cv2.imshow()`. The loop continues until **'q'** is pressed, demonstrating basic video capture and display functionality.

2. `mask.py`

   Expands upon test_camera.py by applying a color mask to isolate specific colors in the video feed. This is useful for tasks like tracking colored objects or navigating using visual markers.

   Code Overview:

   ```python
   # Python
   # Similar setup to test_camera.py
   # Additional OpenCV color transformation and masking
   hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
   blueMin = (100, 150, 0)
   blueMax = (140, 255, 255)
   mask = cv2.inRange(hsv, blueMin, blueMax)
   cv2.imshow("Mask", mask)
   ```

   This script converts the camera feed to HSV color space and applies a mask to detect blue objects, demonstrating how to isolate parts of the image based on color.

3. `blob.py`

   Focuses on identifying and marking blobs (large connected components) in the masked image. This can be used for detecting and analyzing specific shapes or objects in the visual field.

   *Code Overview*:

   ```python
   # Python
   # Using the mask created in mask.py
   detector = cv2.SimpleBlobDetector_create()
   keypoints = detector.detect(mask)
   kp_image = cv2.drawKeypoints(mask, keypoints, None)
   cv2.imshow("Keypoints", kp_image)
   ```

   This script identifies blobs in the masked output and displays these detections, teaching students how to recognize and quantify features in images.

4. `aruco_test.py`

   Introduces the detection of ArUco markers, which are square markers that can be easily recognized and used for various purposes, such as positional tracking and 3D positioning.

   *Code Overview*:

   ```python
   # Python
   import cv2
   from cv2 import aruco

   # Setup camera
   # Initialize ArUco detection
   aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
   parameters = aruco.DetectorParameters_create()
   corners, ids, rejected = aruco.detectMarkers(image, aruco_dict, parameters=parameters)
   image_with_markers = aruco.drawDetectedMarkers(image, corners, ids)
   cv2.imshow("ArUco Markers", image_with_markers)
   ```

   This script processes the video to detect and display ArUco markers, providing a practical application of marker-based navigation or interaction in robotics.

### Lab Session 7: Robot Control with ROS

**Overview**:
Lab 7 introduces students to the Robot Operating System (ROS), a powerful and widely-used open-source robotics middleware. This lab focuses on teaching students how to develop ROS-based applications using Python, specifically covering the creation and management of networked publishers and subscribers.

**Aims and Objectives**:

* **Objective**: To gain hands-on experience with ROS by writing and running publishers and subscribers within the ROS framework.

* **Key Learnings**: Understanding the ROS architecture, managing ROS components, and developing ROS packages using Python and the Catkin build system.

**Contents and Tasks**:

*Task 1: Getting Started*

* **Setup**: Initialize the ROS environment by sourcing the ROS setup script to configure the system's PATH for ROS command usage. Students update their `.bashrc` file to automatically source the ROS environment in every new terminal session.

* **ROS Core**: Start the core ROS services using `roscore &`, which runs in the background for the duration of the lab session.

*Task 2: Running the Example System*

**Turtlesim**: Utilize ROS's `turtlesim` package to understand topics, nodes, publishers, and subscribers. This involves running simulation nodes and nodes that capture key presses to control a simulated turtle.

*Task 3: Examining the System*

* **Investigation Tools**: Use ROS commands like `rosnode list`, `rosnode info`, `rostopic list`, and `rostopic echo` to explore the active ROS system, inspect node details, list topics, and view data being transmitted between nodes.

*Task 4: Creating a Package*

* **Package Setup**: Students create a new ROS package using `catkin_create_pkg`, defining dependencies and setting up the package environment. This includes modifying the `package.xml` for proper metadata and rebuilding the workspace with `catkin_make`.

*Task 5: Examining the Package*

* **Package Configuration**: Ensure that the package setup is correctly sourced using `source` on the workspace's setup file, enabling the package's recognition in the ROS environment.

*Task 6: Writing Some Code*

* **Development**: Write Python scripts for ROS nodes that act as data publishers (`sender.py`) and subscribers (`receiver.py`). This involves setting up topics, publishing and subscribing to messages, and handling ROS node initialization and shutdown.

*Task 7: The Sender and Receiver*

* **Implementation**: Implement and test the sender and receiver scripts. This task includes making Python scripts executable and running them within the ROS environment to observe real-time data transmission between nodes.

*Task 8: Name Clashes*

* **Multiple Instances**: Explore what happens when multiple instances of a node are started and how ROS handles name clashes. Experiment with anonymous node initialization to allow multiple instances.

### Lab 8 Overview: Robot Movement Control

Lab Session 8 in the MSc Practical Robotics and Year 4 Robotics module centers on developing advanced movement control capabilities for a robot. This lab leverages all the skills and tools acquired in previous labs to create a comprehensive movement control program.

**Aims and Objectives**:

* **Objective**: To integrate various components such as sensors, encoders, and motor controls to develop sophisticated movement patterns for a robotic platform.

* **Learning Outcomes**:
   * Develop movement control functions.
   * Utilize sensors and motors for dynamic movement control.
   * Analyze and reproduce specific movement patterns.

**Task Breakdown**:

The Python scripts in Lab 8 for controlling the robot's movement make use of the `TurtleBot` class, a custom Python class designed to abstract and manage the robot's hardware interfaces like motors and sensors. The `Turtlebot` class must be used by other Python programs in the Raspberry Pi while the `generalControl.ino` runs on the Arduino. This Arduino code interfaces with all ARB sensors and actuators, act based on received serial data and send sensor data to serial registers to be read by the Pi.

Below, I'll outline **code snippets** from some of these tasks and explain how they utilize the `TurtleBot` class to execute specific movements.

*Task 1: Create a Movement Control Program*

* **Development**: Enhance existing C or Python code to include functions such as `move_forward(distance)`, `turn_right()`, and `turn_left()` using PWM control statements from previous labs.

* **Functionality**: Implement these functions to control the robot to move accurately in a straight line or turn precisely by 90 degrees. Adjust motor speeds and experiment with different distances to optimize performance and accuracy.

*Task 2: Set Up and Test Repeated Robot Motion*

* **Testing**: Set up a physical space to test the robot's ability to navigate a prescribed 50cm square using markers. Further, program the robot to execute more complex shapes such as lines, rectangles, circles, and figure-eights.

* **Feedback Utilization**: Use feedback from wheel encoders and possibly other sensors to refine the movement accuracy and reliability.

* **Code Implementation**:
   * `task2square.py`: Controls the robot to navigate around a 50cm square, testing the precision of movements and encoder feedback. It uses the `move_forward` method of the `TurtleBot` class to advance a certain distance and the `turn` method to execute 90-degree turns.

      ```python
      from TurtleBot import TurtleBot

      def move_in_square(side_length):
         tb = TurtleBot()
         for _ in range(4):  # Repeat four times to complete a square
            tb.move_forward(side_length)
            tb.turn_right()  # Assuming this method turns the robot 90 degrees to the right

      # Example usage
      move_in_square(50)  # Moves in a 50 cm square
      ```

   * `task2circle.py`: Guides the robot to move in a circular path, focusing on continuous movement control. For circular movement, the script adjusts the speeds of the left and right motors differently to create an arc. The `TurtleBot` class manages these motor speed differences internally.

      ```python
      from TurtleBot import TurtleBot

      def move_in_circle(radius):
         tb = TurtleBot()
         # Assuming a method that sets motor speeds to move in a circle
         tb.move_in_circle(radius)

      # Example usage
      move_in_circle(10)  # Moves in a circle with a specified radius
      ```

   * `task2rectangle.py`: Manages movement in a rectangular trajectory, dealing with different side lengths.

   * `task2eight.py`: Implements control logic for moving the robot in a figure-eight, a complex path that tests the robot's dynamic handling and sensor integration.

*Task 3: Implement Obstacle Detection*

* **Obstacle Interaction**: Integrate the IR sensor to detect obstacles and program the robot to stop within 10cm of any object. Enhance this functionality by programming the robot to navigate around obstacles and resume its intended path.

* **Code Implementation**:

   * `task3.py`: Focuses on obstacle detection and navigation, using the IR sensor to avoid obstacles and modify the robot's path accordingly. This snippet uses the IR sensor functionality of the `TurtleBot` class to detect obstacles and stop or navigate around them. It demonstrates the integration of sensor data into movement decisions:

      ```python
      from TurtleBot import TurtleBot

      def navigate_with_obstacle_detection():
         tb = TurtleBot()
         while True:  # Continuously check for obstacles
            if tb.ir_sensor.detect_obstacle(distance=10):
                  tb.stop()  # Stop if an obstacle is closer than 10 cm
                  tb.turn_right()  # Change direction
                  tb.move_forward(20)  # Move away from the obstacle
            else:
                  tb.move_forward(10)  # Continue moving forward if no obstacle

      # Example usage
      navigate_with_obstacle_detection()
      ```

*Task 4: Error Data Collection & Analysis*

* **Data Collection**: Develop methods to collect and analyze error data related to the robot's movement in straight lines and during rotations. Utilize both IR sensors and encoders to measure errors.

* **Error Analysis**: Compare different methods of measuring movement errors and analyze how errors accumulate based on motor power settings.

### Lab Session 9 Overview: Robot Navigation/Assessment Guidance

Lab Session 9 is the culmination of the robotics module, focusing on integrating various robotic components to achieve complex navigational tasks. This lab involves programming a robot to traverse an obstacle course autonomously, using an array of sensors and actuators developed in previous labs.

**Key Details of Lab 9**:

*Aims and Objectives*:
   * To synthesize knowledge from previous labs into a functional navigation system for a robot.
   * To demonstrate the ability to implement complex behaviors on embedded hardware.
   * To explore the challenges and design considerations for real-world robotic navigation.

*Pre-Lab Preparation*:
   * Students should ensure their robots are fully operational, with all sensors, actuators, and programming from previous labs functioning reliably.
   * Review navigation planning and control methods, focusing on how to utilize the robot's capabilities effectively.

**Key Tasks and Python Scripts Overview**:

*Task 1: Localization*

* Use the robot's camera and IR sensors for basic localization.
* Implement code to allow the robot to identify and navigate towards specific objects or markers using visual cues and distance measurements.

*Task 2: Mapping*

* Develop rudimentary mapping capabilities using the robot's sensors.
* Create either a grid-based or graph-based map that incorporates identified landmarks and the robot's trajectory.

*Task 3: Planning*

* Develop a planning system that enables the robot to navigate through the environment based on the created map and detected landmarks.
* Implement path planning algorithms to efficiently navigate from start to end points.

**Python Scripts Breakdown**:

The solution for the task proposed in this lab can be found under the **UCAS_showcase** directory, explained below. Here are some snippets of some tests that can be found in the **Lab 9** directory:

1. `obstacle_test.py`

This script includes functions to test the robot's obstacle detection and avoidance capabilities.
The robot uses its sensors to detect obstacles in its path and execute maneuvers to navigate around them without human intervention.

*Code Snippet from `obstacle_test.py`*:

   ```python
   # Python
   from TurtleBot import TurtleBot
   import math

   def main():
      tb = TurtleBot(estop=False)
      tb.motors.move_continuous(20)
      tb.motors.rotate_continuous(math.pi)
      tb.motors.move_continuous(20)

   if __name__ == '__main__':
      try:
         main()
      except KeyboardInterrupt:
         print('Interrupted!')
   ```

   The script initializes a `TurtleBot` instance and commands it to move and rotate continuously, testing the integration of movement commands and sensor feedback in real-time.

2. `speed_conversion_new_test.py`

This script is designed to test different speed settings and directions, potentially to calibrate or verify the speed control systems within the robot.

*Code Snippet from `speed_conversion_new_test.py`:*

   ```python
   # Python
   from TurtleBot import TurtleBot
   import time

   def main():
      tb = TurtleBot()
      directions = ["forward", "backward", "left", "right"]
      
      for direction in directions:
         tb.motors.change_direction(direction)
         for speed_level in range(1, 10):
               tb.motors.reset_encoder("A")
               tb.motors.reset_encoder("B")
   ```

   The script tests various movement directions and speed levels, resetting encoder readings between tests to measure the precise effect of each command on the robot's motion.

---

## The "Turtlebot" and "General Control" Classes



---

##  Contributing

Contributions are welcome! Here are several ways you can contribute:

- **[Report Issues](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/issues)**: Submit bugs found or log feature requests for the `rprk_turtlebot_lab_sessions` project.
- **[Submit Pull Requests](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/blob/main/CONTRIBUTING.md)**: Review open PRs, and submit your own PRs.
- **[Join the Discussions](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git/discussions)**: Share your insights, provide feedback, or ask questions.

<details closed>
<summary>Contributing Guidelines</summary>

1. **Fork the Repository**: Start by forking the project repository to your github account.
2. **Clone Locally**: Clone the forked repository to your local machine using a git client.
   ```sh
   git clone https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions.git
   ```
3. **Create a New Branch**: Always work on a new branch, giving it a descriptive name.
   ```sh
   git checkout -b new-feature-x
   ```
4. **Make Your Changes**: Develop and test your changes locally.
5. **Commit Your Changes**: Commit with a clear message describing your updates.
   ```sh
   git commit -m 'Implemented new feature x.'
   ```
6. **Push to github**: Push the changes to your forked repository.
   ```sh
   git push origin new-feature-x
   ```
7. **Submit a Pull Request**: Create a PR against the original project repository. Clearly describe the changes and their motivations.
8. **Review**: Once your PR is reviewed and approved, it will be merged into the main branch. Congratulations on your contribution!
</details>

<details closed>
<summary>Contributor Graph</summary>
<br>
<p align="center">
   <a href="https://github.com{/Alexpascual28/rprk_turtlebot_lab_sessions.git/}graphs/contributors">
      <img src="https://contrib.rocks/image?repo=Alexpascual28/rprk_turtlebot_lab_sessions.git">
   </a>
</p>
</details>

---
