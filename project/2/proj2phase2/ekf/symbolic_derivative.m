
// matlab symbolic derivative script
syms theta0 theta1 theta2 ang_v0 ang_v1 ang_v2 lin_a0 lin_a1 lin_a2 
x1 = cos(theta1)*ang_v0 + sin(theta1)*ang_v2;
x2 = (sin(theta0)*sin(theta1))/cos(theta0)*ang_v0 + ang_v1 + (-(cos(theta1)*sin(theta0))/cos(theta0))*ang_v2;
x3 = -sin(theta1)/cos(theta0)*ang_v0 + cos(theta1)/cos(theta0)*ang_v2;
G_inv_dot = jacobian([x1; x2; x3], [theta0 theta1 theta2])

syms theta0 theta1 theta2 ang_v0 ang_v1 ang_v2 lin_a0 lin_a1 lin_a2 
R1 = [cos(theta2)*cos(theta1)-sin(theta0)*sin(theta1)*sin(theta2), -cos(theta0)*sin(theta2), cos(theta2)*sin(theta1)+cos(theta1)*sin(theta0)*sin(theta2);
     cos(theta1)*sin(theta2)+cos(theta2)*sin(theta0)*sin(theta1), cos(theta0)*cos(theta2), sin(theta2)*sin(theta1)-cos(theta2)*cos(theta1)*sin(theta0);
     -cos(theta0)*sin(theta1), sin(theta0), cos(theta0)*cos(theta1)];
R2 = R1*[lin_a0; lin_a1; lin_a2];
R_dot = jacobian(R2, [theta0 theta1 theta2])


// result
G_inv_dot = 
[                                                                                                                                             0,                                                     ang_v2*cos(theta1) - ang_v0*sin(theta1), 0]
[ ang_v0*sin(theta1) - ang_v2*cos(theta1) - (ang_v2*cos(theta1)*sin(theta0)^2)/cos(theta0)^2 + (ang_v0*sin(theta0)^2*sin(theta1))/cos(theta0)^2, (ang_v0*cos(theta1)*sin(theta0))/cos(theta0) + (ang_v2*sin(theta0)*sin(theta1))/cos(theta0), 0]
[                                               (ang_v2*cos(theta1)*sin(theta0))/cos(theta0)^2 - (ang_v0*sin(theta0)*sin(theta1))/cos(theta0)^2,                       - (ang_v0*cos(theta1))/cos(theta0) - (ang_v2*sin(theta1))/cos(theta0), 0]

R_dot =
[ lin_a1*sin(theta0)*sin(theta2) + lin_a2*cos(theta0)*cos(theta1)*sin(theta2) - lin_a0*cos(theta0)*sin(theta1)*sin(theta2), lin_a2*(cos(theta1)*cos(theta2) - sin(theta0)*sin(theta1)*sin(theta2)) - lin_a0*(cos(theta2)*sin(theta1) + cos(theta1)*sin(theta0)*sin(theta2)), - lin_a0*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta0)*sin(theta1)) - lin_a2*(sin(theta1)*sin(theta2) - cos(theta1)*cos(theta2)*sin(theta0)) - lin_a1*cos(theta0)*cos(theta2)]
[ lin_a0*cos(theta0)*cos(theta2)*sin(theta1) - lin_a2*cos(theta0)*cos(theta1)*cos(theta2) - lin_a1*cos(theta2)*sin(theta0), lin_a2*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta0)*sin(theta1)) - lin_a0*(sin(theta1)*sin(theta2) - cos(theta1)*cos(theta2)*sin(theta0)),   lin_a0*(cos(theta1)*cos(theta2) - sin(theta0)*sin(theta1)*sin(theta2)) + lin_a2*(cos(theta2)*sin(theta1) + cos(theta1)*sin(theta0)*sin(theta2)) - lin_a1*cos(theta0)*sin(theta2)]
[                                     lin_a1*cos(theta0) - lin_a2*cos(theta1)*sin(theta0) + lin_a0*sin(theta0)*sin(theta1),                                                                               - lin_a0*cos(theta0)*cos(theta1) - lin_a2*cos(theta0)*sin(theta1),                                                                                                                                                                                  0]


