# **Example Subsystem Code**

This repository contains example code for multiple subsystems that follow the **FRC 1792 RTR Coding Standard**. It is designed to serve as a reference for implementing various subsystems in a robot project.

---

## **Features**
- **Subsystem Examples**: Includes examples for intake, arm, and elevator (More to be added).
---

## **Project Structure**
- **Subsystems**: Contains reusable subsystem classes for different robot mechanisms.
  - **Arm**:
    - **TalonFX_Arm_WCPSensor_MotionMagic**: Controls a pivoting arm using a TalonFX motor controller with Motion Magic. The arm position is measured using a WCP CANcoder through-bore encoder. Includes methods for moving the arm up, down, stopping, and setting specific positions.
  - **Intake**:
    - **TalonFX_Rollers_WithoutSensor**: Manages the intake rollers without sensor feedback. Includes methods for intaking, outtaking, and stopping the rollers.
    - **TalonFX_Rollers_WithSensor**: Manages the intake rollers with sensor feedback. The sensor stops the rollers when an object is detected. Includes methods for intaking, outtaking, and stopping the rollers.
  - **Elevator**:
    - **TalonFX_Elevator**: Controls an elevator mechanism using two TalonFX motor controllers. Includes methods for moving the elevator up, down, stopping, and setting specific positions. Supports Motion Magic for precise control and includes limit switch handling.

- **Commands**: Includes commands for controlling subsystems with and without sensors.
  - **Arm_Commands**:
    - **Arm_Setpoint_CMD**: Sets the arm to a specific setpoint using Motion Magic. The setpoint is defined in units based on the sensor-to-mechanism ratio.
  - **Intake_Commands**:
    - **Intake_WithoutSensor**:
      - **Roller_Intake_CMD**: Activates the intake rollers to collect game pieces without sensor feedback.
      - **Roller_Outtake_CMD**: Activates the intake rollers to release game pieces without sensor feedback.
    - **Intake_WithSensor**:
      - **Roller_Intake_WithSensor_CMD**: Activates the intake rollers to collect game pieces. The rollers stop automatically when the sensor detects an object.
      - **Roller_Outtake_WithSensor_CMD**: Activates the intake rollers to release game pieces. This command does not rely on sensor feedback.
  - **Elevator_Commands**:
    - **Elevator_Setpoint_CMD**: Moves the elevator to a specific setpoint using Motion Magic. The setpoint is defined in rotations and can be adjusted dynamically.

---
