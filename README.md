# 6-DOF KUKA-Style Robotic Arm

This is an end-to-end open-source project to design, simulate, and build a 6-DOF (Degrees of Freedom) robotic arm inspired by the KUKA industrial robot.

The project is a public journal following a 19-step roadmap, covering the entire professional engineering pipeline from initial theory to a fully functional, ROS-controlled physical robot.



## Project Phases

This repository is organized according to the 5-phase project plan:

1.  **Phase 1: Design, Kinematics & Core Theory**
    * Conceptual Design & CAD.
    * Kinematic Modeling (DH Parameters).
    * Forward & Inverse Kinematics (FK/IK)
    * Jacobian, Singularities & Velocity Control.
    * Dynamics Modeling.
    * MATLAB GUI Application.

2.  **Phase 2: Simulation & Path Planning**
    * Trajectory Planning.
    * URDF Generation.
    * Multi-Platform Simulation.

3.  **Phase 3: Advanced Control & HIL**
    * Control System Theory.
    * Simulink & Simscape Model.
    * Model-in-the-Loop (MIL) / Hardware-in-the-Loop (HIL).

4.  **Phase 4: ROS Integration & Virtual Deployment**
    * ROS Environment Setup.
    * RViz & MoveIt! Setup.
    * Gazebo Simulation Integration.

5.  **Phase 5: Physical Build & ROS Deployment**
    * Fabrication & Assembly.
    * Hardware Interface.
    * Final System Deployment.
    * Teleoperation / "Sync" Mode.

## Tech Stack

* **CAD:** SolidWorks
* **Kinematics & Control Design:** MATLAB, Simulink, Simscape
* **Robotic OS:** ROS (Robot Operating System)
* **Simulation:** Gazebo, CoppeliaSim, RViz , RoboDK
* **Path Planning:** MoveIt!
* **Hardware:** Raspberry Pi, Stepper Motors, 3D Printed Parts
* **Core Programming:** Python, C++