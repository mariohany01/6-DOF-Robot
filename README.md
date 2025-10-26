# 6-DOF KUKA-Style Robotic Arm

This is an end-to-end open-source project to design, simulate, and build a 6-DOF (Degrees of Freedom) robotic arm inspired by the KUKA industrial robot.

The project is a public journal following a 19-step roadmap, covering the entire professional engineering pipeline from initial theory to a fully functional, ROS-controlled physical robot.



## Project Phases

This repository is organized according to the 5-phase project plan:

1.  **Phase 1: Design, Kinematics & Core Theory**
    * Developing the 3D model in SolidWorks.
    * Deriving Denavit-Hartenberg (DH) parameters.
    * Implementing Forward/Inverse Kinematics, Jacobian, and Dynamics in MATLAB.

2.  **Phase 2: Simulation & Path Planning**
    * Generating smooth trajectories and creating a URDF model.
    * Simulating the robot in CoppeliaSim to validate the model.

3.  **Phase 3: Advanced Control & HIL**
    * Designing and tuning PID controllers in a high-fidelity Simulink & Simscape model.
    * Validating the control logic in real-time with Hardware-in-the-Loop (HIL) testing.

4.  **Phase 4: ROS Integration & Virtual Deployment**
    * Porting the robot into the ROS ecosystem.
    * Configuring MoveIt! for advanced motion planning and RViz for visualization.
    * Integrating with the Gazebo physics simulator for a full "digital twin."

5.  **Phase 5: Physical Build & ROS Deployment**
    * Fabricating (3D printing) and assembling the physical arm.
    * Developing the `ros_control` hardware interface to link ROS to the physical motors.
    * Deploying final features like real-time teleoperation ("Sync" mode) and "Record & Playback."

## Tech Stack

* **CAD:** SolidWorks
* **Kinematics & Control Design:** MATLAB, Simulink, Simscape
* **Robotic OS:** ROS (Robot Operating System)
* **Simulation:** Gazebo, CoppeliaSim, RViz
* **Path Planning:** MoveIt!
* **Hardware:** Raspberry Pi, Stepper Motors, 3D Printed Parts
* **Core Programming:** Python, C++