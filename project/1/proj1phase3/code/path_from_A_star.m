% Waypoint Generator Using the A*   
% Optimal_path = path_from_A_star(test_map);
% where Optimal_path is feeded to trajectory_generator()
function Optimal_path = path_from_A_star(map)
    %Optimal_path = [];
    size_map = size(map,1);

    MAX_X=10;
    MAX_Y=10;
    
    % Define the 2D grid map array.
    % 0: free path, 1: obstacles
    % start and target point are passed as input parameters
    % use int8 instead of default double for efficiency
    MAP = int8(zeros(MAX_X,MAX_Y));
    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1));
    yval=floor(map(size_map, 2));
    xTarget=xval;
    yTarget=yval;
    
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1
        xval=floor(map(i, 1));
        yval=floor(map(i, 2));
        MAP(xval,yval)=1;
    end 
    
    %Initialize MAP with location of the start point
    xval=floor(map(1, 1));
    yval=floor(map(1, 2));
    xStart=xval;
    yStart=yval;
    
    %start your code
    GoalRegister = int8(zeros(MAX_X,MAX_Y));
    GoalRegister(xTarget,yTarget)=1;
    Connecting_Distance = 1;
    Optimal_path = AstarPath(xStart,yStart,MAP,GoalRegister,Connecting_Distance); 
    
end
