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

on the Raspberry Pi:

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

1. Clone the repository to your local machine or download the entire project directory.

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

2. Install the ARB Library on your Arduino:
   1. In the *Arduino IDE*, go to the menu bar and select **Sketch** > **Include Library** > **Add .ZIP Library....**
   2. Zip the ARB directory and navigate to where you have saved your **"ARB.zip"** file.
   3. Select the file and click on **'Open'**. The IDE will then install the library.
   4. Verify Installation:
      - To check if the library has been successfully installed, go back to **Sketch** > **Include Library**. You should see the library named "ARB" at the bottom of the drop-down menu.
      - Click on it to include the library in your current sketch, which should automatically insert an include statement like `#include <ARB.h>` at the top of your sketch.

3. Open a connection to the Raspberry Pi using PuTTY or directly through HDMI. (Windows instructions). Further instructions in [Lab 1](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions/blob/main/Lab1/autumn_lab_1.pdf):
   1. Connect to the Raspberry Pi with your laptop using the serial **USB to UART HAT** and a USB cable.
   2. Check what *COM* port the device is connected to using "Device Manager"
   3. Establish a `Serial` connection with PuTTY using the device *COM* port and baud rate 115200.
   
   Alternatively, you can connect a screen and keyboard directly to the Raspberry Pi to access the terminal directly.

   4. Login using your login details for the Raspberry Pi in CLI. For the lab RPRK devices, the details are the following:
      * **SSID:** *pi*
      * **Password:** *raspberry*

4. Connect the Pi to a local WiFi network and check the device's IP address on the network.
   1. Type `sudo raspi-config` in the command line to open the configuration screen.
   2. Go to **“2: Network Options”** and then **“N2 Wireless LAN”** and enter the SSID and passphrase for your network.
   3. Go to "Finish" and wait a few moments for the Raspberry Pi to connect.
   4. Type `ifconfig` on the terminal.
   5. Look for the section called *wlan0*. You should see your IP address there (e.g 144.32.70.210).
   6. Take note of your IP, it can be used to connect to the board through SSH or to transfer files with FTP. You can now close the serial PuTTY or direct connection.

5. Connect through SSH using PuTTY (Windows instructions)
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

6. View, add and modify files using WinSCP (Windows instructions). Further instructions in [Lab 1](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions/blob/main/Lab1/autumn_lab_1.pdf)
   1. Open WinSCP
   2. Create a "New Site" with the following details:
      * **File Protocol**: *SFTP*
      * **Host Name**: The device's IP address for the network in format *XXX.XX.XX.XXX* (e.g *144.32.70.210*). Refer to step 4 in [Installation](#installation).
      * **Port**: *22*
      * **User Name**: Your SSID for the Raspberry Pi. In lab devices: ***pi***.
      * **Password**: Your password for the Raspberry Pi. In lab devices: ***raspberry***.

###  Usage

<h4>From <code>source</code></h4>

> Run rprk_turtlebot_lab_sessions using the command below:
> ```console
> $ python main.py
> ```

###  Tests

> Run the test suite using the command below:
> ```console
> $ pytest
> ```

---

##  Project Roadmap

- [X] `► INSERT-TASK-1`
- [ ] `► INSERT-TASK-2`
- [ ] `► ...`

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

##  License

This project is protected under the [SELECT-A-LICENSE](https://choosealicense.com/licenses) License. For more details, refer to the [LICENSE](https://choosealicense.com/licenses/) file.

---

##  Acknowledgments

- List any resources, contributors, inspiration, etc. here.

[**Return**](#-overview)

---
