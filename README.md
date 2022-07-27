# Robotics Motion Planning
<<<<<<< HEAD
## introduction

In this project, a MATLAB program is written to plan a trajectory for the end-effector of a mobile manipulator robot called youBot (a mobile base with four meconium wheels and a 5R robot arm), to complete a task that youBot pick up a cube at an initial location, move and place the cube at a desired position. The final output of the program is a .csv file, it will be imported into the CoppeliaSim Edu to simulate youBot to achieve the task.

The video link below is to show how the robot work in the CoppeliaSim Edu.
https://youtu.be/eVqjFB_kyz8

## Describtion

In the project, we separated codes into four parts, they are ‘TrajectoryGenerator’, ‘NextState’, ‘FeedbackControl’ and ‘WrapperCode’ respectively. These functions are described as below.

### 1.	TrajectoryGenerator
The TrajectoryGenerator was used to create a group of reference trajectory for the end-effector of the youBot. This trajectory consists of eight concatenated trajectory segments and the whole process was finished in 15s. To avoid instability and high speed of movement in the long distance, the first segment (move the gripper from its initial configuration to a ‘standoff’ configuration a few cm above the block) was finished in 4s and sixth segment (move the gripper to the final configuration of the object) was finished in 5s, the other six segment were assigned with 1s each.

Since the time step (∆t) in this project is set as 0.01s and k =1, so there are 100 reference configurations every second. The configurations in each segment were created by the ‘CartesianTrajectory’ function from the Modern Robotics code library.

### 2.	NextState
In this part, the function ‘NextState’ uses the kinematics of the youBot, knowledge of velocity kinematics, and the Euler method to predict how the robot will move in a small timestep ∆t based on its current configuration and velocity. The function would take the current state of the robot, current joint and wheel angular velocity, timestep ∆t and maximum angular velocity of joint and wheel as input, and output the next state of the robot.

One important part needs to be mentioned that if any current joint or wheel angular velocity is more than maximum velocity, then the current joint or wheel angular velocity will be replaced by maximum velocity (the direction of rotation would remain the same). This velocity limit is used to improve stability of movement, excessive speed may cause cube drop during the movement. However, in practice, the angular velocity would also be limited by robot’s characters.

### 3.	FeedbackControl

The function ‘FeedbackControl’, which calculates the task-space feedforward plus feedback control law. It would output u (angular velocity of wheels) and θ ̇ (angular velocity of arm joint), which are used to move current configuration to next desired configuration in the a small timestep ∆t.
  
One thing in our ‘FeedbackControl’ function needs to be mentioned is that our function would take the Xe list (a 6*N matrix used to store error every loop, N is an integer and would increase as the times of ‘FeedbackControl’ function being used), and it outputs the an updated Xe list including the new error just being generated in the ‘FeedbackControl’ function.

### 4.	WrapperCode

In the ‘WrapperCode’, we combine all the previous functions to generate an actual trajectory of the robot. The initial actual configuration was set as [0 -0.5 0 0 1.2 -1.5 -0.5 0.5 0 0 0 0], which fulfill the requirement from the guideline (at least 30 degrees of orientation error and 0.2 m of position error).

The first step is to use ‘TrajectoryGenerator’ to generate the desired trajectory of the robot for reference. Then the program would go through N-1 loops (N is 1500 in this project) using ‘FeedbackControl’ and ‘NextState’ function to calculate the rest of configurations followed by the initial actual configuration we set. Finally, a .csv file store actual trajectory of the robot and the figure of error through the whole process would be generated.

Besides, the Kp and Ki in the ‘WrapperCode’ can be changed or optimized based on the animation and the figure of errors.



## Appendix
The programs above called many functions from 'Modern Robotics code library' attached with the book 'Modern Robotics: Mechanics, Planning, and Controlt' written by Kevin M. Lynch and Frank C. Park. Modern Robotics code library can be downloaded from http://hades.mech.northwestern.edu/index.php/Modern_Robotics

=======
>>>>>>> eb10166 (first commit)
