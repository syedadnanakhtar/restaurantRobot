parameters.timestep = 0.05; 
parameters.robotRadius = 0.1;

load img.mat
map = createMap([0, 0], 0.05, img);%first argument is the origin, second is the resolution (m/pixel)

% Create Dijkstra Map Layouts
%Locations of points and Graph of Layout 1 (standard 2way layout):
pointLocations1 = [5 40; 30 40; 30 25 ;70 40;70 25;5 60;30 60;30 75;70 60;70 75];
connections1{1} = [2 ;25];
connections1{2} = [1 3 4 7;25 15 40 35];
connections1{3} = [2;15];
connections1{4} = [2 5 9;40 15 35];
connections1{5} = [4;15];
connections1{6} = [7 ;25 ];
connections1{7} = [2 6 8 9;35 25 15 40];
connections1{8} = [7;15];
connections1{9} = [4 7 10;35 40 15];
connections1{10} =[9;15];
map1 = {pointLocations1,connections1};

%Roundabout Map = map2:
pointLocations2 = [5 40; 30 40; 30 25 ;70 40;70 25;5 60;30 60;30 75;70 60;70 75];
connections2{1} = [2 ; 25];
connections2{2} = [3 4 ; 15 40];
connections2{3} = [5 ;40];
connections2{4} = [9; 35];
connections2{5} = [4 ; 15];
connections2{6} = [1 ;100 ];
connections2{7} = [2 6 ;35 25];
connections2{8} = [7; 15];
connections2{9} = [7 10; 40 15];
connections2{10} =[8;40];
map2 = {pointLocations2,connections2};

%% Set the map
mapp = map1;

%Plot the map
plotMap(map);
hold on
xlabel('y');
ylabel('x');
chk_pt = [0.25 2;1.5 2;1.5 1.25;3.5 2;3.5 1.25;0.25 3;1.5 3;1.5 3.75;3.5 3;3.5 3.75];

start1 = chk_pt(1,:);
start2 = chk_pt(6,:);

%Define checkpoints where the robot has to pass. The final row is the goal.
path1 = [Astar(mapp,start1(1),start1(2),10);Astar(mapp,chk_pt(10,1),chk_pt(10,2),1)];%[0.25 2;1.5 2;1.5 3;1.5 3.75];
path2 = [Astar(mapp,start2(1),start2(2),5);Astar(mapp,chk_pt(5,1),chk_pt(5,2),6)];%;[0.25 3;1.5 3;1.5 2;1.5 1.25];


%Define initial robot state
robotState1.x = path1(1,1); 
robotState1.y = path1(1,2);
robotState1.heading = 0;
robotState1.vel = 0.0;
robotState1.omega = 0.0;

robotState2.x = path2(1,1); 
robotState2.y = path2(1,2);
robotState2.heading = 0;
robotState2.vel = 0.0;
robotState2.omega = 0.0;
EndFlag1 = 0; %becomes one when the robot has reached the goal or has gone out of the map..
EndFlag2 = 0;
i= 1;
j = 1;
eps = 0.5;
active = ones(size(path1,1),1);
active(1) = 0;
traj1 = zeros(0,3);
traj2 = zeros(0,3);

plot(robotState1.y, robotState1.x, 'og', 'MarkerFaceColor', 'g','LineWidth',2);
plot(robotState2.y, robotState2.x, 'or', 'MarkerFaceColor', 'r','LineWidth',0.1);
legend('Robot 1','Robot 2');
labels={'Start 1','Start 2','Goal 1','Goal 2'};
text([path1(1,2)+0.2 path2(1,2)+0.2 path1(end,2)-0.5 path2(end,2)],[path1(1,1) path2(1,1) path1(end,1)+0.25 path2(end,1)+0.25],labels);%,'VerticalAlignment','bottom','HorizontalAlignment','Right');
title(['Theta1: ' num2str(rad2deg(robotState1.heading)) '| Theta2: ' num2str(rad2deg(robotState2.heading))]);
set(0,'DefaultLegendAutoUpdate','off')
plot([path1(1,2) path2(1,2) path1(end,2) path2(end,2)],[path1(1,1) path2(1,1) path1(end,1) path2(end,1)], 'o','MarkerSize',12);

while (i<=size(path1,1) || j<=size(path2,1)) && (~EndFlag1 || ~EndFlag2)
    set(0,'DefaultLegendAutoUpdate','off')
    % Move Robot 1
    if ~EndFlag1
    %First rotate the robot1 towards goal
    while robotState1.heading ~= atan2(path1(i,2)-robotState1.y,abs(path1(i,1)-robotState1.x))
        omega = sign(atan2(path1(i,2)-robotState1.y,robotState1.x-path1(i,1)) - robotState1.heading ) * deg2rad(60);
        robotState1 = updateRobotState(robotState1, 0, omega, parameters.timestep);
        %plot(robotState1.y, robotState1.x, 'og', 'MarkerFaceColor', 'g'); 
        title(['Theta1: ' num2str(rad2deg(robotState1.heading)) '| Theta2: ' num2str(rad2deg(robotState2.heading))]);
        set(0,'DefaultLegendAutoUpdate','off')
    end
    % After the robot is oriented towards the next checkpoint, give it a translational velocity
    if (robotState1.x ~= path1(i,1) || robotState1.y ~= path1(i,2))
        if(robotState1.x > path1(i,1))
            v = -1;
        else
            v = 1;
        end
        robotState1 = updateRobotState(robotState1, v, 0, parameters.timestep);
    end
    %disp(['x1:' num2str(robotState1.x) '| y1:' num2str(robotState1.y) '| theta1:' num2str(rad2deg(robotState1.heading))])
    end
    
    % Move Robot 2
    if ~EndFlag2
    %First rotate the robot2 towards goal
    while robotState2.heading ~= atan2(path2(j,2)-robotState2.y,abs(path2(j,1)-robotState2.x))
        omega = sign(atan2(path2(j,2)-robotState2.y,abs(robotState2.x-path2(j,1))) - robotState2.heading ) * deg2rad(60);
        robotState2 = updateRobotState(robotState2, 0, omega, parameters.timestep);
        %plot(robotState2.y, robotState2.x, 'or', 'MarkerFaceColor', 'r'); 
        title(['Theta1: ' num2str(rad2deg(robotState1.heading)) '| Theta2: ' num2str(rad2deg(robotState2.heading))]);
        
    end
    % After the robot is oriented towards the next checkpoint, give it a translational velocity
    if norm([robotState2.x-path2(j,1) robotState2.y-path2(j,2)]) == eps
        disp(['Stopping at: ' num2str(robotState2.x) ', ' num2str(robotState2.y)]);
        m = find((path1(:,1) == path2(j,1) & path1(:,2) == path2(j,2))==1);
       
        if active(m)==1
            v = 0;
            robotState2 = updateRobotState(robotState2, v, 0, parameters.timestep);
        elseif (robotState2.x ~= path2(j,1) || robotState2.y ~= path2(j,2))
            if(robotState2.x > path2(j,1))
            v = -1;
        else
            v = 1;
        end
            robotState2 = updateRobotState(robotState2, v, 0, parameters.timestep);
        end
    elseif (robotState2.x ~= path2(j,1) || robotState2.y ~= path2(j,2))
         if(robotState2.x > path2(j,1))
            v = -1;
        else
            v = 1;
        end
         robotState2 = updateRobotState(robotState2, v, 0, parameters.timestep);
    end
%     if (robotState2.x ~= path2(j,1) || robotState2.y ~= path2(j,2))
%         v = 1;
%         robotState2 = updateRobotState(robotState2, v, 0, parameters.timestep);
%     end
    %disp(['x2:' num2str(robotState2.x) '| y2:' num2str(robotState2.y) '| theta2:' num2str(rad2deg(robotState2.heading))])
    end

  
    
    % plot the robot position
    %traj1 = [traj1; robotState1.x robotState1.y robotState1.heading];
    %traj2 = [traj2; robotState2.x robotState2.y robotState2.heading];
    plot(robotState1.y, robotState1.x, 'og', 'MarkerFaceColor', 'g','LineWidth',2);
    plot(robotState2.y, robotState2.x, 'or', 'MarkerFaceColor', 'r','LineWidth',0.1); 
    title(['Theta1: ' num2str(rad2deg(robotState1.heading)) '| Theta2: ' num2str(rad2deg(robotState2.heading))]);
 
    %Check if checkpoint has been reached.
    if (robotState1.x == path1(i,1) && robotState1.y == path1(i,2) && i<size(path1,1))
        active(i) = 0;
        i = i + 1; 
        disp('Checkpoint reached for Robot 1');
    end
    if (robotState2.x == path2(j,1) && robotState2.y == path2(j,2) && j<size(path2,1))
        j = j + 1;
        disp('Checkpoint reached for Robot 2');
    end
    
    %Check if Goal has been reached.  
    if isequal(path1(end,:),[robotState1.x robotState1.y]) && i == size(path1,1) && EndFlag1 ~= true
        EndFlag1 = true;
        disp('Goal 1 Reached.')
    end
    if isequal(path2(end,:),[robotState2.x robotState2.y]) && j == size(path2,1) && EndFlag2 ~= true
        EndFlag2 = true;
        disp('Goal 2 Reached.')
    end
    
    %Check if robot is going out of bound
    if robotState1.x < 0 || robotState1.x > 5 || robotState1.y<0 || robotState1.y>5 || robotState2.x < 0 || robotState2.x > 5 || robotState2.y<0 || robotState2.y>5
        EndFlag1 = true;
        EndFlag2 = true;
        disp('One of the Robots went outside the allowed area.');
    end
    drawnow;
    %w = waitforbuttonpress;
    pause(parameters.timestep);
end

