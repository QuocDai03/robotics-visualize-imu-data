# 🧭 MPU6050 → ROS 2 → RViz2 Orientation Visualization

## 1. 📖 Overview

This project demonstrates a simple integration between an **Embedded system** and **ROS 2**.

A **MPU6050 IMU sensor** is connected to an **STM32F103C8T6 (Blue Pill)** microcontroller. The MCU reads orientation-related data from the MPU6050 and sends the data to a laptop running **ROS 2 Humble**.

ROS 2 then processes the received IMU data and visualizes the orientation of a 3D object in **RViz2**.

The main goal of **Phase 1** is to visualize the inclination of the IMU around the **Roll** and **Pitch** axes.

### 🔄 System Pipeline

```text
┌──────────────┐
│   MPU6050    │
│     IMU      │
└──────┬───────┘
       │ I²C
       ▼
┌──────────────┐
│     STM32    │
│  Blue Pill   │
│ STM32F103C8T6│
└──────┬───────┘
       │ Serial / USB
       ▼
┌──────────────────┐
│      ROS 2       │
│     Humble       │
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│      RViz2       │
│  Visualization   │
└──────────────────┘
```

The expected result is:

> 🖐️ When the physical MPU6050 is tilted, the corresponding 3D object in RViz2 will rotate accordingly.

---

## 2. ✨ Feature

### 🎯 Visualization

Visualize the **Roll** and **Pitch** orientation of the MPU6050 in RViz2.

The system should allow the user to:

* 📐 Read IMU data from the MPU6050.
* 🔄 Determine the **Roll** angle.
* 🔄 Determine the **Pitch** angle.
* 📡 Transfer IMU data from the STM32 to the laptop.
* 🤖 Publish the data through ROS 2.
* 🖥️ Visualize the corresponding orientation of a 3D object in RViz2.

### 📊 Target Output

```text
Roll  :  +15.2°
Pitch :   -7.8°
```

and a corresponding 3D object in RViz2:

```text
             Z
             ↑
             │
        ┌────┼────┐
       /    /     /
      /    /     /
     └────┴─────┘
          ───────→ X
```

The object should change its orientation according to the physical movement of the MPU6050.

> ⚠️ **Scope of Phase 1:** Only Roll and Pitch are considered. Yaw, sensor fusion, position tracking, and physics simulation are outside the scope of this phase.

---

## 3. 🛠️ Technologies

### 3.1 💻 Software

| Technology                | Purpose                                      |
| ------------------------- | -------------------------------------------- |
| 🐧 **Ubuntu Linux**       | Development environment for ROS 2            |
| 🤖 **ROS 2 Humble**       | Robot middleware and communication framework |
| 👁️ **RViz2**             | 3D visualization                             |
| 💻 **Visual Studio Code** | ROS 2 / C++ / Python development             |
| 🔧 **STM32CubeIDE**       | STM32 firmware development                   |
| 🔌 **STM32 HAL**          | MCU peripheral abstraction                   |
| 🌿 **Git / GitHub**       | Source code management                       |

### ROS 2

**ROS 2 Humble** is used as the main robotics middleware.

It will be responsible for:

* 📡 Receiving data from the STM32.
* 🔄 Processing / converting IMU data.
* 📢 Publishing data through ROS 2 topics.
* 🧭 Providing orientation information for visualization.
* 👁️ Connecting the orientation data to RViz2.

### RViz2

**RViz2** is used to visualize the orientation of the physical IMU.

The visualization will focus on:

* 🧭 Coordinate frames.
* 📐 Roll.
* 📐 Pitch.
* 🧊 3D object orientation.

---

### 3.2 🔩 Hardware

| Hardware                            | Description                                                                |
| ----------------------------------- | -------------------------------------------------------------------------- |
| 🧭 **MPU6050**                      | 6-axis IMU with accelerometer and gyroscope                                |
| 🔲 **STM32F103C8T6 Blue Pill**      | Main microcontroller                                                       |
| 🔌 **PL2303 USB-to-UART Converter** | Converts the STM32 UART interface to USB for communication with the laptop |
| 💻 **Laptop / PC**                  | Runs ROS 2 Humble and RViz2                                                |

### MPU6050

The **MPU6050** is used as the project's IMU sensor.

It provides:

* 📈 3-axis accelerometer
* 🔄 3-axis gyroscope
* 🔌 I²C communication interface

For Phase 1, the main focus is obtaining the orientation around:

```text
Roll  → Rotation around X-axis

Pitch → Rotation around Y-axis
```

### STM32F103C8T6

The **STM32F103C8T6 Blue Pill** acts as the embedded controller.

Its main responsibilities are:

```text
MPU6050
   │
   │ I²C
   ▼
STM32F103C8T6
   │
   │ UART
   ▼
PL2303
   │
   │ USB
   ▼
Laptop
```

The firmware will:

1. ⚙️ Initialize the MCU peripherals.
2. 🔌 Initialize the I²C interface.
3. 🧭 Initialize the MPU6050.
4. 📥 Read IMU data.
5. 📐 Calculate Roll and Pitch.
6. 📤 Transmit the IMU data through the STM32 UART interface.

### PL2303 USB-to-UART Converter

The **PL2303** is used as a bridge between the STM32 UART interface and the laptop's USB interface.

Its role is:

```text
STM32 UART
    │
    │ UART protocol
    ▼
  PL2303
    │
    │ USB
    ▼
 Laptop / PC
```

The PL2303 does **not** process the IMU data. It only converts the electrical/interface format between **UART** on the STM32 side and **USB** on the laptop side.

The laptop receives the UART data through a USB serial device such as:

```text
/dev/ttyUSB0
```

### Laptop / PC

The laptop runs the ROS 2 environment and performs the data processing and visualization:

```text
STM32
  │
  │ UART
  ▼
PL2303
  │
  │ USB
  ▼
Laptop
  │
  │ ROS 2
  ▼
/imu/data
  │
  ▼
RViz2
  │
  ▼
3D Orientation Visualization
```

---


## 🎯 Phase 1 Goal

The Phase 1 implementation is considered successful when:

> **Physically tilting the MPU6050 causes a corresponding Roll/Pitch rotation of the 3D object displayed in RViz2.**

```text
        PHYSICAL WORLD                    ROS 2 WORLD

     ┌───────────┐
     │  MPU6050  │
     │    🧭     │
     └─────┬─────┘
           │
        tilt ↗
           │
           ▼
     ┌───────────┐
     │   STM32   │
     └─────┬─────┘
           │
           │ IMU data
           ▼
       ┌───────┐
       │ ROS 2 │
       └───┬───┘
           │
           ▼
     ┌────────────┐
     │   RViz2    │
     │            │
     │   🧊 ↗     │
     │  3D Object │
     └────────────┘
```

### 🚧 Future Phases

The following features are intentionally **not included in Phase 1**:

* ⏳ Yaw estimation
* 🧮 Sensor fusion
* 🧭 Quaternion-based orientation estimation
* 🌐 Full 3D orientation
* 🏭 Gazebo simulation
* 📍 Position / displacement tracking
* 🤖 Robot model integration
* 📊 Advanced data visualization

These features can be introduced in later phases as the project evolves.
