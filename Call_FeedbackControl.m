clear
clc

X=[0, 0, 0, 0, 0, 0.2, -1.6, 0];
Tsed = [0 0 1 0.5; 0 1 0 0; -1 0 0 0.5; 0 0 0 1];
Tsedn = [0 0 1 0.6; 0 1 0 0; -1 0 0 0.3; 0 0 0 1];
Kp = 0*eye(6);
Ki = 0*eye(6);
delt_t = 0.01;

u_thetadot = FeedbackControl(X,Tsed,Tsedn,Kp,Ki,delt_t)
