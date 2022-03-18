# MAE204_Robotics_FinalProj
<<<<<<< HEAD
## introduction

Final project in the MAE204 Robotics, a robotic will pick up a cube from initial configuration to final configuration

The video link below is to show how the program work in the CoppeliaSim Edu

https://youtu.be/U1PvPN91wcM
## Describtion

In the project, we separated codes into four parts, they are ‘TrajectoryGenerator’, ‘NextState’, ‘FeedbackControl’ and ‘WrapperCode’ respectively. These functions are described as below.

### 1.	TrajectoryGenerator
The TrajectoryGenerator was used to create a group of reference trajectory for the end-effector of the youBot. This trajectory consists of eight concatenated trajectory segments and the whole process was finished in 15s. To avoid instability and high speed of movement in the long distance, the first segment (move the gripper from its initial configuration to a ‘standoff’ configuration a few cm above the block) was finished in 4s and sixth segment (move the gripper to the final configuration of the object) was finished in 5s, the other six segment were assigned with 1s each.

Since the time step (∆t) in this project is set as 0.01s and k =1, so there are 100 reference configurations every second. The configurations in each segment were created by the ‘CartesianTrajectory’ function from the Modern Robotics code library.

### 2.	NextState
In this part, the function ‘NextState’ uses the kinematics of the youBot, knowledge of velocity kinematics, and the Euler method to predict how the robot will move in a small timestep ∆t based on its current configuration and velocity [1]. The function would take the current state of the robot, current joint and wheel angular velocity, timestep ∆t and maximum angular velocity of joint and wheel as input, and output the next state of the robot.

One important part needs to be mentioned that if any current joint or wheel angular velocity is more than maximum velocity, then the current joint or wheel angular velocity will be replaced by maximum velocity (the direction of rotation would remain the same). This velocity limit is used to improve stability of movement, excessive speed may cause cube drop during the movement. However, in practice, the angular velocity would also be limited by robot’s characters.

## Appendix

mr file is the function download from , which is attached with the book 'Modern Robot' written by 

=======
>>>>>>> eb10166 (first commit)
