Project Overview


This project focuses on automating the pick-and-place task of a disk in a manufacturing process using a robotic arm (manipulator). The disk arrives at Station 1 from a conveyor, is picked by the robotic arm, and placed at Station 2 for labeling. After labeling, the robotic arm picks the disk from Station 2 and places it at Station 3, where the disk is transferred to another conveyor for the next process.


Task Requirements:
•	Object (Disk): 2 cm width, 0.5 kg weight
•	Stations: The disk is placed at the center of each station
•	Tools:
o	RoboDK for simulation of the robotic arm/manipulator
o	MATLAB for trajectory planning and velocity profiling


Robotic Arm Selection
When selecting a robotic arm from the RoboDK library, the following factors are taken into account:
•	Degrees of Freedom (DoF): Sufficient DoF to handle complex movements in the pick-and-place task.
•	Reach (Workspace): The robotic arm must be capable of reaching all stations with ease.
•	Payload Capacity: The arm should be able to handle the weight of the disk (0.5 kg).


Waypoints for Trajectory Planning
In this project, two waypoints are selected between the starting point (pick) and the endpoint (place) to simplify the trajectory planning. These waypoints are used to simulate the smooth movement of the robotic arm.



Simulation Using RoboDK
The selected robotic arm is simulated in RoboDK, where:
•	The robotic arm picks the disk from Station 1.
•	Places it at Station 2 for labeling.
•	Finally, moves the disk to Station 3, where it is transferred to another conveyor.




Trajectory Planning and Kinematics with MATLAB
Using MATLAB (with and without the Robotics Toolbox), the following tasks are performed:
•	Forward Kinematics: Calculated to verify the positional relationship of the robotic arm.
•	Inverse Kinematics: Verified to determine the joint parameters needed for the end effector to reach the required positions at the waypoints.
•	Velocity Profile: Generated to ensure smooth motion and efficient task execution.





Conclusion
This project successfully automates the pick-and-place task using a robotic arm for a simplified manufacturing process. The simulation in RoboDK and trajectory planning in MATLAB allow for accurate task execution and real-world implementation in industrial automation.

