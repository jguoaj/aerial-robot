function s_des = circle_trajectory(t, true_s)
    
    s_des = zeros(13,1);

    omega=25;      
    x_des=4*cos(t*omega/180*pi);
    y_des=4*sin(t*omega/180*pi);
    z_des=3/25*t;    % since the duration of the trajectory is 25 s

    x_vdes=-omega/180*pi*4*sin(t*omega/180*pi);   
    y_vdes= omega/180*pi*4*cos(t*omega/180*pi);
    z_vdes=3/25;           

    s_des(1)=x_des;
    s_des(2)=y_des;
    s_des(3)=z_des;
    s_des(4)=x_vdes;
    s_des(5)=y_vdes;
    s_des(6)=z_vdes;
    
    %desired yaw angle in the flight
    des_yaw = mod(0.1 * pi * t,2 * pi);
    ypr = [des_yaw, 0.0, 0.0];              % yaw pitch row
    Rot = ypr_to_R(ypr);                    % ratation matrix
    q_des = R_to_quaternion(Rot);           % R_to_quaternion changes 9-entry rotation matrix into 4-entry rotation
    s_des(7:10) = q_des;
end
