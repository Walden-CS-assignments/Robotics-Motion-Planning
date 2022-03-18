%*****input*****
%The current actual end-effector configuration X (aka Tse) 
%X = [phi, x, y, theta1, theta2, theta3, theta4, theta5,U1,U2,U3,U4]
%The current reference end-effector configuration Xd (aka Tse,d)
%The reference end-effector configuration at the next timestep, Xd,next (aka Tse,d,next)
%The PI gain matrices Kp (6x6) and Ki (6x6)
%The timestep ∆t between reference trajectory configurations
%The record of previous err Xe_list
%*****output*****
%The commanded end-effector twist V expressed in the end-effector frame {e}
%The updated record of err Xe_list

function [u_thetadot,Xe_list] = FeedbackControl(X,Tsed,Tsedn,Kp,Ki,delt_t,Xe_list)
    %Transfer X to Tse
    phi = X(1,1); x = X(1,2); y = X(1,3);
    Tsb = [cos(phi) -sin(phi) 0 x; sin(phi) cos(phi) 0 y; 0 0 1 0.0963;...
        0 0 0 1];

    Tb0 = [1 0 0 0.1662; 0 1 0 0; 0 0 1 0.0026; 0 0 0 1];
    M0e = [1 0 0 0.033; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1];
    Blist = [[0; 0; 1; 0; 0.033; 0], [0; -1; 0; -0.5076; 0; 0], ...
            [0; -1; 0; -0.3526; 0; 0],[0;-1;0;-0.2176;0;0],[0;0;1;0;0;0]];
    %get thetalist from current configuration X
    thetalist = X(1, 4:8)';
   
    %Calculate T0e
    T0e= FKinBody(M0e,Blist,thetalist);
    
    %Thus, the Tse will be 
    Tse = Tsb*Tb0*T0e;

    %Calculate the feedforward reference twist Vd (6x1) from Xd to Xd,next
    Vd = se3ToVec((1/delt_t)*MatrixLog6(inv(Tsed)*Tsedn));

    %Calculate the feedforward twist Vd in the actual end-effect frame at
    %Tse, which is Vd_a (6x1)
    Vd_a = Adjoint(inv(Tse)*Tsed)*Vd;

    %Calculate the Xe (twist, 6x1) that take Tse to Tsed in unit time
    Xe = se3ToVec(MatrixLog6(inv(Tse)*Tsed));
    
    Xe_acu=sum(Xe_list,2);
    %Calculate the Commanded end-effector twist V expressed in the 
    %end-effector frame {e}
    Vb = Vd_a+Kp*Xe+Ki*Xe_acu*0.01;
    %Update the new error to the Xe_list
    Xe_list=[Xe_list,Xe];

%*******Calculate Je******
    %Calculate Jbase
        l = 0.235; w = 0.15; r = 0.0475;
        
        F = r/4 * [-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w); 1 1 1 1; -1 1 -1 1];
        F6 = [0 0 0 0; 0 0 0 0; F; 0 0 0 0];
        
        %finally,get Jbase
        Jbase = Adjoint(inv(T0e)*inv(Tb0))*F6;

    %Calculate Jarm
    Jarm = JacobianBody(Blist,thetalist);

    %Combine Jbase and Jarm
    Je = [Jbase Jarm];

    %add the tolerance for the pseudoinverse algorithm, avoid unreasonably 
    % large entries in the Jacobian psuedoinverse, which could lead to 
    % unacceptably large commanded joint speeds
    pseudoinv_wTole_Je = pinv(Je,1e-2);
    
    %Calculate the wheel angular speed u and arm joint angular speed
    %thetadot, !!! u_thetadot is a 9*1 matrix
    u_thetadot = pseudoinv_wTole_Je*Vb;
    %then make u_thetadot transpose and it become 1*9 matrix
    u_thetadot = transpose(u_thetadot);
     
end