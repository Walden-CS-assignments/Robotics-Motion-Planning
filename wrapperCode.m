%********Input********
%1. the initial resting configuration of the cube frame, Tsc_intial
%2. the desired final resting configuration of the cube frame, Tsc_final
%3. the actual initial configuration of the youBot, 
% X_intial = [chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5]
%4. the reference initial configuration of the youBot (different from
%actual to test feedback control)
%5. gains for your feedback controller, Kp and Ki
%*******output********
%1. a .csv file (13x1500), each line has 13 entries as below
%chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state
%2. a data file containing the 6-vector end-effector error as a function of time

function para = wrapperCode(Tsc_intial, Tsc_final, X_intial, Tsed_intial , Kp, Ki)

    %Calculate the reference trajectory
    Tce_grasp = [-sqrt(2)/2 0 sqrt(2)/2 0; 0 1 0 0; ... 
        -sqrt(2)/2 0 -sqrt(2)/2 0; 0 0 0 1];
    Tce_standoff = [-sqrt(2)/2 0 sqrt(2)/2 0; 0 1 0 0; ...
        -sqrt(2)/2 0 -sqrt(2)/2 0.25; 0 0 0 1];
    k = 1;
    
    %assume intial wheel angles are all 0
    X = X_intial;
    %The gripper will be open at the beginning
    para = [X 0];
    
    %Generator a Refernce Trajectory (1500*13)
    Ref_Traj = TrajectoryGenerator(Tsed_intial, Tsc_intial, Tsc_final, ...
        Tce_grasp, Tce_standoff, k);
    
    for i = 1:1499
        %Based on the current X and Tsed (1-1499) and Tsedn (2-1500) to 
        %calculate the u and thetadot

        Tsed = [Ref_Traj(i,1:3) Ref_Traj(i, 10); Ref_Traj(i,4:6) ...
            Ref_Traj(i, 11); Ref_Traj(i, 7:9) Ref_Traj(i,12); 0 0 0 1];

        Tsedn = [Ref_Traj(i+1,1:3) Ref_Traj(i+1, 10); Ref_Traj(i+1,4:6) ...
            Ref_Traj(i+1, 11); Ref_Traj(i+1, 7:9) Ref_Traj(i+1,12); 0 0 0 1];
        
        %Define delt_t and Max Angular Velocity
        delt_t = 0.01; MaxVel = 2;

        %get the velocity from current configuration to next configuration
        %!!!only the first eight parameters of X will be used here 
        %excluding wheel angles
        u_thetadot = FeedbackControl(X(1,1:8),Tsed,Tsedn,Kp,Ki,delt_t);
        
        %Next configuration X_next would be calculated, 1*12
        X_next = NextState(X, u_thetadot, delt_t, MaxVel);
        
        %store X_next into para, gripper status will depend on the next
        %reference trajectory's gripper status
        para = [para; X_next Ref_Traj(i+1,13)];

        %Next configuration become current configuration, then do next loop
        X = X_next;      
    end
    csvwrite('wrapper.csv',para);
end
    



