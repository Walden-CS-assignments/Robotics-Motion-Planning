% ******input****** 
% X = [phi x y theta1 theta2 theta3 theta4 theta5 U1 U2 U3 U4]
% u_thetadot = [u1 u2 u3 u4 thetadot1 thetadot2 thetadot3 thetadot4 thetadot5]
% timestep size: delt_t
% The maximum joint and wheel velocity magnitude: maxVel
% *******output*******
% [nx ny nphi ntheta1 ntheta2 ntheta3 ntheta4 ntheta5 U1 U2 U3 U4]

%Input
X = [0 0 0 0 0 0 0 0 0 0 0 0];
u_thetadot = [8 8 8 8 0 -1 -0.5 -1 0];
delt_t = 0.01;
maxVel = 2;

%intialization, and gripper is open(0)
para = [X 0];

for i = 1:100
    nextstate = NextState(X, u_thetadot, delt_t, maxVel);
    %the gripper will keep open(0)
    para = [para; nextstate 0];
    %After time of delt_t, the nextstate will become current configuration
    X = nextstate;
end

csvwrite('nextstate_para.csv',para);


