%% Path Generation
path = [zeros(12,1) (-2.2:0.1:-1.1)' 0*ones(12,1)];
path = [path; (0.1:0.1:1.1)' (-1.0:0.1:0)' 45*ones(11,1)];

%% V-REP Simulation Exercise 3: Kinematic Control
% Tests the implemented control algorithm within a V-Rep simulation.

% In order to run the simulation:
%   - Start V-Rep
%   - Load the scene matlab/common/vrep/mooc_exercise.ttt
%   - Hit the run button
%   - Start this script

%% Parameters setup
 
%% Define parameters for Dijkstra and Dynamic Window Approach
parameters.dist_threshold= 0.25; % threshold distance to goal
parameters.angle_threshold = 0;%0.1; % threshold orientation to goal

%% Initialize connection with V-Rep
startup;
connection = simulation_setup();
connection = simulation_openConnection(connection, 0);
simulation_start(connection);

%% Get static data from V-Rep
bob_init(connection);

parameters.wheelDiameter = bob_getWheelDiameter(connection);
parameters.wheelRadius = parameters.wheelDiameter/2.0;
parameters.interWheelDistance = bob_getInterWheelDistance(connection);
parameters.scannerPoseWrtBob = bob_getScannerPose(connection);

% controller parameters
parameters.Krho = 0.5;
parameters.Kalpha = 1.5;
parameters.Kbeta = -0.6;
parameters.backwardAllowed = true;
parameters.useConstantSpeed = true;
parameters.constantSpeed = 0.4;

bob_setTargetGhostPose(connection, -0,-1.1, 90);
bob_setTargetGhostVisible(connection, 1);



%% CONTROL LOOP.
EndCond = 0;
i=2;
while (~EndCond && i<=23)
    %% CONTROL STEP.
    % Get pose and goalPose from vrep
    [x, y, theta] = bob_getPose(connection);
    %[xg, yg, thetag] = bob_getTargetGhostPose(connection);
    disp(['x:' num2str(x) '| y:' num2str(y) '| theta:' num2str(theta)])
    xg = path(i,1);
    yg = path(i,2);
    thetag = path(i,3);
    
    % run control step
    omega = 0;%normalizeAngle( angleDiff(theta,atan2(yg-y,xg-x)) );
    vu = parameters.constantSpeed; %sqrt( (yg-y)^2 + (xg-x)^2 );

    % Calculate wheel speeds
    [LeftWheelVelocity, RightWheelVelocity ] = calculateWheelSpeeds(vu, omega, parameters);
    
    % End condition
    
    %dtheta = abs(normalizeAngle(theta-thetag));

    %rho = sqrt((xg-x)^2+(yg-y)^2);  % pythagoras theorem, sqrt(dx^2 + dy^2)
    %EndCond = (rho < parameters.dist_threshold && dtheta < parameters.angle_threshold) || rho > 5; 
    
    EndCond = sqrt( (path(end,2)-y)^2 + (path(end,1)-x)^2 ) < 0.1;
    
    if x == path(i,1) && y == path(i,2) % && ~EndCond
        disp('Reached a checkpoint');
        i = i+1;
    end
    
    % SET ROBOT WHEEL SPEEDS.
    %bob_setWheelSpeeds(connection, LeftWheelVelocity, RightWheelVelocity);
    bob_setWheelSpeeds(connection,LeftWheelVelocity, RightWheelVelocity);
end

%% Bring Bob to standstill
bob_setWheelSpeeds(connection, 0.0, 0.0);

simulation_stop(connection);
simulation_closeConnection(connection);

msgbox('Simulation ended');
