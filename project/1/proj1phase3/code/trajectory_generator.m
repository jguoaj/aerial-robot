function s_des = trajectory_generator(t, path, h, map)
persistent p
persistent t_vec
persistent T

if nargin > 1 % pre-process can be done here (given waypoints). Pre-define the entire trajectory.

% visualize the 2D grid map
subplot(h);
% start point
plot3(map(1, 1)-0.5, map(1, 2)-0.5, map(1, 3)-0.5, 'k.');
hold on;
% obstacles
for obs_cnt = 2: size(map, 1) - 1
    plot3([map(obs_cnt, 1)-0.2 map(obs_cnt, 1)-0.8], [map(obs_cnt, 2)-0.2 map(obs_cnt, 2)-0.8], [map(obs_cnt, 3) map(obs_cnt, 3)], 'k-');
    hold on;
    plot3([map(obs_cnt, 1)-0.2 map(obs_cnt, 1)-0.8], [map(obs_cnt, 2)-0.8 map(obs_cnt, 2)-0.2], [map(obs_cnt, 3) map(obs_cnt, 3)], 'k-');
    hold on;
    [X,Y,Z] = cylinder(0.4);
    Z(2, :) = 5;
    X(:, :) = X(:, :) + map(obs_cnt, 1) - 0.5;
    Y(:, :) = Y(:, :) + map(obs_cnt, 2) - 0.5;
    mesh(X,Y,Z,'edgecolor', [0.7, 0.7, 0.7], 'facecolor', [0.7,0.7,0.7]); 
    grid minor
    set(gca,'xtick',[-100:1:100])
    set(gca,'ytick',[-100:1:100])
    grid off;
    grid on;       
    axis equal;        
    axis ([-1 6 -1 10 0 4]);
    hold on;
end
% target point
plot3(map(obs_cnt+1, 1)-0.5, map(obs_cnt+1, 2)-0.5, map(obs_cnt+1, 3)-0.5, 'r*');
hold on;

%%% preprocessing
%path=[0 0 0; path];
path_z = ones(length(path),1);
path = horzcat(path, path_z);
path = flipud(path)
m = length(path) - 1;  % eg. 12-1=11

%%% segment duration according to the 2-norm between each point
distance_vec = zeros(1, m);
for i = 1:m
    distance_vec(i) = norm(path(i+1,:) - path(i,:));
end
total_distance = sum(distance_vec);
t_vec = zeros(1, m+1);   % time vector
T = zeros(1, m);         % time duration of each intervals
t_vec(1) = 0;
for i = 1:m
    t_vec(i+1) = t_vec(i) + 25 * (distance_vec(i)/total_distance);
end
for i = 1:m
    T(i) = t_vec(i+1) - t_vec(i);
end

%%% cost function
Q = zeros(8*m,8*m);
for t=1:m
    for i=4:7
        for k=4:7
            Q(8*t-i,8*t-k)=i*(i-1)*(i-2)*(i-3)*k*(k-1)*(k-2)*(k-3)*(T(t)^(i+k-7))/(i+k-7);
        end
    end
end 

%%% derivative constraint
dimention = 2*m + 4*(m-1) + 3 + 3;
d = zeros(dimention, 3);
A = zeros(dimention, 8*m);
% position constraint
for i=1:m
    d(2*i-1,1:3) = path(i,1:3);
    d(2*i,1:3)   = path(i+1,1:3);
end
for i=1:m
    A(2*i-1:2*i,8*i-7:8*i)=[
        0,0,0,0,0,0,0,1;...
        T(i)^7, T(i)^6, T(i)^5, T(i)^4, T(i)^3, T(i)^2, T(i)^1, 1
    ];
end

%%% continuity constraint
% explanation: page 16 of lec4, [Aj -Aj+1] 
% for Aj, time is T(i), for Aj+1, time is 0
for i=1:(m-1)
   A(2*m+4*i-3:2*m+4*i,(8*i-7):(8*i+8))=[
       T(i)^7, T(i)^6, T(i)^5, T(i)^4, T(i)^3, T(i)^2, T(i)^1, 1,          0,0,0,0,0,0,0,-1;...
       7*T(i)^6,6*T(i)^5, 5*T(i)^4, 4*T(i)^3, 3*T(i)^2, 2*T(i)^1, 1, 0,    0,0,0,0,0,0,-1,0;...
       42*T(i)^5,30*T(i)^4, 20*T(i)^3, 12*T(i)^2, 6*T(i)^1, 2, 0, 0,       0,0,0,0,0,-2,0,0;...
       210*T(i)^4,120*T(i)^3, 60*T(i)^2, 24*T(i)^1, 6, 0, 0, 0             0,0,0,0,-6,0,0,0;
       ]; 
end
% start point velocity, acceleration, jerk constraint
A((6*m-3):(6*m-1),1:8)=[
    0, 0, 0, 0, 0, 0, 1, 0;...
    0, 0, 0, 0, 0, 2, 0, 0;...
    0, 0, 0, 0, 6, 0, 0, 0;
];
% end point velocity, acceleration, jerk constraint
A((6*m):(6*m+2),(8*m-7):(8*m))=[
    7*T(m)^6,6*T(m)^5, 5*T(m)^4, 4*T(m)^3, 3*T(m)^2, 2*T(m)^1, 1, 0;...
    42*T(m)^5,30*T(m)^4, 20*T(m)^3, 12*T(m)^2, 6*T(m)^1, 2, 0, 0;...
    210*T(m).^4,120*T(m).^3, 60*T(m).^2, 24*T(m).^1, 6*T(m).^0, 0, 0, 0;
];

%%% quadratic programming
% x = quadprog(H,f,A,b,Aeq,beq) solves the preceding problem subject to 
% the additional restrictions Aeq*x = beq. 
% Aeq is a matrix of doubles, and beq is a vector of doubles. 
% If no inequalities exist, set A = [] and b = [].
f = zeros(8*m,1);
p1 = quadprog(Q,f,[],[],A,d(:,1));
p2 = quadprog(Q,f,[],[],A,d(:,2));
p3 = quadprog(Q,f,[],[],A,d(:,3));
p = [p1 p2 p3];

% state: eg. s_des = zeros(13,1);
% [1:6]: x, y, z, x_v, y_v, z_v
% [7:10]: quaternion, ratation representation
% [11:13]: wx, wy, wz
else % output desired trajectory here (given time)
    s_des = zeros(13,1);
    index = 1;  time = 0;
    for i = 1:length(T)  % length(T) = m
        if t>=t_vec(i) && t<=t_vec(i+1)
            index = i;
            time = t_vec(i);
            break
        end
    end
    time_d = t - time;
    p_t = p(8*index-7:8*index,1:3);
    pos_poly = [time_d^7, time_d^6, time_d^5, time_d^4, time_d^3, time_d^2, time_d, 1];
    vel_poly = [7*time_d^6, 6*time_d^5, 5*time_d^4, 4*time_d^3, 3*time_d^2, 2*time_d, 1, 0];
    
    s_des(1:3) = pos_poly * p_t;
    s_des(4:6) = vel_poly * p_t;
    
    ypr = [0.0, 0.0, 0.0];
    s_des(7:10) = R_to_quaternion(ypr_to_R(ypr));
end

end


