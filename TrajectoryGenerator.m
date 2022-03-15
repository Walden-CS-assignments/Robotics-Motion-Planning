function EEtrajectory = TrajectoryGenerator(Tse_intial, Tsc_intial, Tsc_final,Tce_grasp, Tce_standoff, k)
    %every step cost 1s
    t1 = 1; t2 = 2; t3 = 3; 
    N1 = t1*k/0.01;
    N2 = t2*k/0.01;
    N3 = t3*k/0.01;
    method = 5;
    %1st step (intial place to standoff place before grasp), gripper status
    %= 0, time = 3s
    Tse_intialstandoff = Tsc_intial*Tce_standoff;
    traj_1 = CartesianTrajectory(Tse_intial, Tse_intialstandoff, t3, N3, method);
    for i = 1:N3
        if i == 1
            para = [traj_1{i}(1, 1:3) traj_1{i}(2,1:3) traj_1{i}(3, 1:3) traj_1{i}(1:3,4)' 0];
            continue
        end
        para = [para; traj_1{i}(1, 1:3) traj_1{i}(2,1:3) traj_1{i}(3, 1:3) traj_1{i}(1:3,4)' 0];
    end
    
    %2nd step (standoff to cube), gripper status = 0, time = 2s
    Tse_initialgrasp = Tsc_intial*Tce_grasp;
    traj_2 = CartesianTrajectory(Tse_intialstandoff, Tse_initialgrasp, t2, N2, method);
    for i = 1:N2
        para = [para; traj_2{i}(1, 1:3) traj_2{i}(2,1:3) traj_2{i}(3, 1:3) traj_2{i}(1:3,4)' 0];
    end

    
    %3rd step (close the gripper), gripper status = 1, time = 1
    traj_3 = CartesianTrajectory(Tse_initialgrasp, Tse_initialgrasp, t1, N1, method);
    for i = 1:N1
        para = [para; traj_3{i}(1, 1:3) traj_3{i}(2,1:3) traj_3{i}(3, 1:3) traj_3{i}(1:3,4)' 1];
    end
    
    %4th step (back to the intial standoff), gripper status = 1, time = 2s
    traj_4 = CartesianTrajectory(Tse_initialgrasp, Tse_intialstandoff, t2, N2, method);
    for i = 1:N2
        para = [para; traj_4{i}(1, 1:3) traj_4{i}(2,1:3) traj_4{i}(3, 1:3) traj_4{i}(1:3,4)' 1];
    end
    
    %5th step (to the final standoff), gripper status = 1, time = 3s
    Tse_finalstandoff = Tsc_final*Tce_standoff;
    traj_5 = CartesianTrajectory(Tse_intialstandoff, Tse_finalstandoff, t3, N3, method);
    for i = 1:N3
        para = [para; traj_5{i}(1, 1:3) traj_5{i}(2,1:3) traj_5{i}(3, 1:3) traj_5{i}(1:3,4)' 1];
    end
    
    %6th step (to final cube), gripper status = 1, time = 2s
    Tse_finalgrasp = Tsc_final*Tce_grasp;
    traj_6 = CartesianTrajectory(Tse_finalstandoff, Tse_finalgrasp, t2, N2, method);
    for i = 1:N2
        para = [para; traj_6{i}(1, 1:3) traj_6{i}(2,1:3) traj_6{i}(3, 1:3) traj_6{i}(1:3,4)' 1];
    end
    
    %7th step (open the gripper), gripper status = 0
    traj_7 = CartesianTrajectory(Tse_finalgrasp, Tse_finalgrasp, t1, N1, method);
    for i = 1:N1
        para = [para; traj_7{i}(1, 1:3) traj_7{i}(2,1:3) traj_7{i}(3, 1:3) traj_7{i}(1:3,4)' 0];
    end

    %8th step (beack to the finalstandoff), gripper status = 0;
    traj_8 = CartesianTrajectory(Tse_finalgrasp, Tse_finalstandoff, t1, N1, method);
    for i = 1:N1
        para = [para; traj_8{i}(1, 1:3) traj_8{i}(2,1:3) traj_8{i}(3, 1:3) traj_8{i}(1:3,4)' 0];
    end
    EEtrajectory = para;

    csvwrite('trajectorypara.csv',para);

end