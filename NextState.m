% ******input****** 
% X = [phi x y theta1 theta2 theta3 theta4 theta5 U1 U2 U3 U4]
% chasis: phi x y, arm joint angles: theta(1-5), wheel angles: U(1-4)
% u_thetadot = [u1 u2 u3 u4 thetadot1 thetadot2 thetadot3 thetadot4
% thetadot5]
% wheel angular velocity: u(1-4), joint angular velocity: thetadot(1-5)
% timestep size: delt_t
% The maximum joint and wheel velocity magnitude: maxVel

% *******output*******
% next state of robot after time of delt_t = 
% [nx ny nphi ntheta1 ntheta2 ntheta3 ntheta4 ntheta5 U1 U2 U3 U4]

function nextstate = NextState(X, u_thetadot, delt_t, maxVel)
    
    %*******current configuration*********
    phi=X(1,1); x=X(1,2); y=X(1,3); theta1=X(1,4); theta2=X(1,5);
    theta3=X(1,6); theta4=X(1,7); theta5=X(1,8); U1=X(1,9); U2=X(1,10);
    U3=X(1,11); U4=X(1,12);
    %*******current velocity**********
    u1 = u_thetadot(1,1);  u2 = u_thetadot(1,2);  u3 = u_thetadot(1,3);
    u4 = u_thetadot(1,4); thetadot1 = u_thetadot(1,5); 
    thetadot2 = u_thetadot(1,6); thetadot3 = u_thetadot(1,7);
    thetadot4 = u_thetadot(1,8); thetadot5 = u_thetadot(1,9);

    %joint angles in the next state after time of delt_t
    ntheta1 = theta1 + thetadot1*delt_t;
    ntheta2 = theta2 + thetadot2*delt_t;
    ntheta3 = theta3 + thetadot3*delt_t;
    ntheta4 = theta4 + thetadot4*delt_t;
    ntheta5 = theta5 + thetadot5*delt_t;

    %wheel angles in the next state after time of delt_t
    nU1 = U1 + u1*delt_t;
    nU2 = U2 + u2*delt_t;
    nU3 = U3 + u3*delt_t;
    nU4 = U4 + u4*delt_t;
    
    %parameters of the chassis
    l = 0.235; w = 0.15; r = 0.0475;
    
    %Calculate the body twist
    F = r/4 * [-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w); 1 1 1 1; -1 1 -1 1];
    delt_theta = [u1*delt_t; u2*delt_t; u3*delt_t; u4*delt_t];
    Vb = F*delt_theta;

    wbz = Vb(1,1);
    Vbx = Vb(2,1);
    Vby = Vb(3,1);
    if wbz == 0
        delt_qb = [0; Vbx; Vby];
    else
        delt_qb = [wbz; (Vbx*sin(wbz)+Vby*(cos(wbz)-1))/wbz; ...
            (Vby*sin(wbz)+Vbx*(1-cos(wbz)))/wbz];
    end

    %tranferm delt_qb in {b} to delt_q in {s}
    delt_q = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)]*delt_qb;

    %[phi; x; y] is the current coordinate
    % qnext(3*1) is the next coordinate after time of delt_t
    qnext = [phi; x; y] + delt_q;
   
    %nextstate(1*12) is the configuration after time of delt_t
    nextstate = [qnext' ntheta1 ntheta2 ntheta3 ntheta4 ntheta5 nU1 nU2 nU3 nU4];

end