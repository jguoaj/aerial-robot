% input: (time, current_state, des_state)
% output: force F, moment M 

% state: eg. s_des = zeros(13,1);
% [1:6]: x, y, z, x_v, y_v, z_v
% [7:10]: quaternion, ratation representation
% [11:13]: wx, wy, wz

function [F, M] = controller(t, s, s_des)

global params

m = params.mass;
g = params.grav;
I = params.I;

% hyper parameters: kp and kd
k_p_yaw = 15;      k_d_yaw = 10;
k_p_pitch = 100;   k_d_pitch = 30;
k_p_row = 100;     k_d_row = 30;

k_p_p1 = 4;    k_d_p1 = 6;
k_p_p2 = 4;    k_d_p2 = 6;
k_p_p3 = 200;  k_d_p3 = 100;

%%% position control
R = quaternion_to_R(s(7:10));
[phi, theta, psi] = RotToRPY_ZXY(R);  % page 26 of lecture 2, rotation matrix to angles
R_des = quaternion_to_R(s_des(7:10));
[phi_des, theta_des, psi_des] = RotToRPY_ZXY(R_des); 

% desired accelation, page 40 of lecture 2
% linearized version
p1_dot_dot = g * (theta_des*cos(psi_des) + phi_des*sin(psi_des));
p2_dot_dot = g * (theta_des*sin(psi_des) - phi_des*cos(psi_des));
p3_dot_dot = 0;
% non-linearized version
% p1_dot_dot = 0 + R_des(1,:)*[0;0;g];
% p2_dot_dot = 0 + R_des(2,:)*[0;0;g];
% p3_dot_dot = -g + R_des(3,:)*[0;0;g];

% page 41 of lecture 2, PID control
p1_c_dot_dot = p1_dot_dot + k_d_p1*(s_des(4)-s(4)) + k_p_p1*(s_des(1)-s(1));
p2_c_dot_dot = p2_dot_dot + k_d_p2*(s_des(5)-s(5)) + k_p_p2*(s_des(2)-s(2));
p3_c_dot_dot = p3_dot_dot + k_d_p3*(s_des(6)-s(6)) + k_p_p3*(s_des(3)-s(3));

% calculat u1(F) and commanded angles
F = m * (g + p3_c_dot_dot);
phi_c = (p1_c_dot_dot*sin(psi) - p2_c_dot_dot*cos(psi))/g;
theta_c = (p1_c_dot_dot*cos(psi) + p2_c_dot_dot*sin(psi))/g;
psi_c = psi_des;


%%% attitude control
% PID
ypr_dot_dot = [k_p_row * (phi_c - phi) + k_d_row * (s_des(11) - s(11)); ...
               k_p_pitch * (theta_c - theta) + k_d_pitch * (s_des(12) - s(12)); ...
               k_p_yaw * (psi_c - psi) + k_d_yaw * (s_des(13) - s(13))];

% euler equation
W = [s(11);s(12);s(13)];
M = I * ypr_dot_dot + cross(W, I*W);

% F = 1.0; M = [0.0, 0.0, 0.0]'; % You should calculate the output F and M

end











