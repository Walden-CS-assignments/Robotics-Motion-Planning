%1. the initial resting configuration of the cube frame, Tsc_intial
%2. the desired final resting configuration of the cube frame, Tsc_final
%3. the actual initial configuration of the youBot, 
% X_intial = [chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5]
%4. the reference initial configuration of the youBot (different from
%actual to test feedback control)
%5. gains for your feedback controller, Kp and Ki
clc
clear

Tsc_intial = [1 0 0 1; 0 1 0 0; 0 0 1 0.025; 0 0 0 1];
Tsc_final = [0 1 0 0; -1 0 0 -1; 0 0 1 0.025; 0 0 0 1];
%a random actual initial configuration
X_intial = [0 0.2 0 0.2 0 0 0.2 0 0 0 0 0];

Tsed_intial = [0 0 1 0; 0 1 0 0; -1 0 0 0.5; 0 0 0 1];
Kp = 0*eye(6);
Ki = 0*eye(6);

para = wrapperCode(Tsc_intial, Tsc_final, X_intial, Tsed_intial, Kp, Ki);