%***********Input*****************
%The initial configuration of the end-effector: Tse,initial
%The initial configuration of the cube: Tsc,initial
%The desired final configuration of the cube: Tsc,f inal
%The configuration of the end-effector relative to the cube while grasping: Tce,grasp
%The standoff configuration of the end-effector above the cube, before and 
% after grasping, relative to the cube:Tce,standof f
%The number of trajectory reference configurations per 0.01 seconds: k. 
% The value k is an integer with a value of 1 or greater.
%************Output*******************
%A representation of the N configurations of the end-effector along the entire 
% concatenated eight-segment reference trajectory. Each of these N reference 
% points represents a transformation matrix Tse of the end-effector frame {e} 
% relative to {s} at an instant in time, plus the gripper state (0 for open or 1 for closed)
%A .csv file with the entire eight-segment reference trajectory. Each line of 
% the .csv file corresponds to one configuration Tse of the end-effector, 
% expressed as 13 variables separated by commas. The 13 variables are, in order:
%r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper state

function EEtrajectory = TrajectoryGenerator(Tse_intial, Tsc_intial, Tsc_final,Tce_grasp, Tce_standoff, k)
    %every step cost 1s
    t1 = 1; t2 = 2; t4 = 4; t5 = 5;
    N1 = t1*k/0.01; N2 = t2*k/0.01; N4 = t4*k/0.01; N5 = t5*k/0.01;
    method = 5;
    %1st step (intial place to standoff place before grasp), gripper status
    %= 0, time = 4s
    Tse_intialstandoff = Tsc_intial*Tce_standoff;
    traj_1 = CartesianTrajectory(Tse_intial, Tse_intialstandoff, t4, N4, method);
    for i = 1:N4
        if i == 1
            para = [traj_1{i}(1, 1:3) traj_1{i}(2,1:3) traj_1{i}(3, 1:3) traj_1{i}(1:3,4)' 0];
            continue
        end
        para = [para; traj_1{i}(1, 1:3) traj_1{i}(2,1:3) traj_1{i}(3, 1:3) traj_1{i}(1:3,4)' 0];
    end
    
    %2nd step (standoff to cube), gripper status = 0, time = 1s
    Tse_initialgrasp = Tsc_intial*Tce_grasp;
    traj_2 = CartesianTrajectory(Tse_intialstandoff, Tse_initialgrasp, t1, N1, method);
    for i = 1:N1
        para = [para; traj_2{i}(1, 1:3) traj_2{i}(2,1:3) traj_2{i}(3, 1:3) traj_2{i}(1:3,4)' 0];
    end

    
    %3rd step (close the gripper), gripper status = 1, time = 1s
    traj_3 = CartesianTrajectory(Tse_initialgrasp, Tse_initialgrasp, t1, N1, method);
    for i = 1:N1
        para = [para; traj_3{i}(1, 1:3) traj_3{i}(2,1:3) traj_3{i}(3, 1:3) traj_3{i}(1:3,4)' 1];
    end
    
    %4th step (back to the intial standoff), gripper status = 1, time = 1s
    traj_4 = CartesianTrajectory(Tse_initialgrasp, Tse_intialstandoff, t1, N1, method);
    for i = 1:N1
        para = [para; traj_4{i}(1, 1:3) traj_4{i}(2,1:3) traj_4{i}(3, 1:3) traj_4{i}(1:3,4)' 1];
    end
    
    %5th step (to the final standoff), gripper status = 1, time = 5s
    Tse_finalstandoff = Tsc_final*Tce_standoff;
    traj_5 = CartesianTrajectory(Tse_intialstandoff, Tse_finalstandoff, t5, N5, method);
    for i = 1:N5
        para = [para; traj_5{i}(1, 1:3) traj_5{i}(2,1:3) traj_5{i}(3, 1:3) traj_5{i}(1:3,4)' 1];
    end
    
    %6th step (to final cube), gripper status = 1, time = 1s
    Tse_finalgrasp = Tsc_final*Tce_grasp;
    traj_6 = CartesianTrajectory(Tse_finalstandoff, Tse_finalgrasp, t1, N1, method);
    for i = 1:N1
        para = [para; traj_6{i}(1, 1:3) traj_6{i}(2,1:3) traj_6{i}(3, 1:3) traj_6{i}(1:3,4)' 1];
    end
    
    %7th step (open the gripper), gripper status = 0, time = 1s
    traj_7 = CartesianTrajectory(Tse_finalgrasp, Tse_finalgrasp, t1, N1, method);
    for i = 1:N1
        para = [para; traj_7{i}(1, 1:3) traj_7{i}(2,1:3) traj_7{i}(3, 1:3) traj_7{i}(1:3,4)' 0];
    end

    %8th step (beack to the finalstandoff), gripper status = 0, time = 1s
    traj_8 = CartesianTrajectory(Tse_finalgrasp, Tse_finalstandoff, t1, N1, method);
    for i = 1:N1
        para = [para; traj_8{i}(1, 1:3) traj_8{i}(2,1:3) traj_8{i}(3, 1:3) traj_8{i}(1:3,4)' 0];
    end
    EEtrajectory = para;

    csvwrite('trajectorypara.csv',para);

end