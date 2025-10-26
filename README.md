\# 6-DOF KUKA-Style Robotic Arm



This is an end-to-end open-source project to design, simulate, and build a 6-DOF (Degrees of Freedom) robotic arm inspired by the KUKA industrial robot.



The project is a public journal following a 19-step roadmap, covering the entire professional engineering pipeline from initial theory to a fully functional, ROS-controlled physical robot.







\## Project Phases



This repository is organized according to the 5-phase project plan:



1\.  \*\*Phase 1: Design, Kinematics \& Core Theory\*\*

&nbsp;   \* Developing the 3D model in SolidWorks.

&nbsp;   \* Deriving Denavit-Hartenberg (DH) parameters.

&nbsp;   \* Implementing Forward/Inverse Kinematics, Jacobian, and Dynamics in MATLAB.



2\.  \*\*Phase 2: Simulation \& Path Planning\*\*

&nbsp;   \* Generating smooth trajectories and creating a URDF model.

&nbsp;   \* Simulating the robot in CoppeliaSim to validate the model.



3\.  \*\*Phase 3: Advanced Control \& HIL\*\*

&nbsp;   \* Designing and tuning PID controllers in a high-fidelity Simulink \& Simscape model.

&nbsp;   \* Validating the control logic in real-time with Hardware-in-the-Loop (HIL) testing.



4\.  \*\*Phase 4: ROS Integration \& Virtual Deployment\*\*

&nbsp;   \* Porting the robot into the ROS ecosystem.

&nbsp;   \* Configuring MoveIt! for advanced motion planning and RViz for visualization.

&nbsp;   \* Integrating with the Gazebo physics simulator for a full "digital twin."



5\.  \*\*Phase 5: Physical Build \& ROS Deployment\*\*

&nbsp;   \* Fabricating (3D printing) and assembling the physical arm.

&nbsp;   \* Developing the `ros\_control` hardware interface to link ROS to the physical motors.

&nbsp;   \* Deploying final features like real-time teleoperation ("Sync" mode) and "Record \& Playback."



\## Tech Stack



\* \*\*CAD:\*\* SolidWorks

\* \*\*Kinematics \& Control Design:\*\* MATLAB, Simulink, Simscape

\* \*\*Robotic OS:\*\* ROS (Robot Operating System)

\* \*\*Simulation:\*\* Gazebo, CoppeliaSim, RViz

\* \*\*Path Planning:\*\* MoveIt!

\* \*\*Hardware:\*\* Raspberry Pi, Stepper Motors, 3D Printed Parts

\* \*\*Core Programming:\*\* Python, C++

