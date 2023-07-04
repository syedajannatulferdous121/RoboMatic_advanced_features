% RoboMatic - Advanced MATLAB Robotics Project

% Define robot parameters
robotLength = 0.5;  % Robot length in meters
robotWidth = 0.3;   % Robot width in meters

% Initialize robot position and orientation
robotX = 0;         % Robot X position in meters
robotY = 0;         % Robot Y position in meters
robotTheta = 0;     % Robot orientation in radians

% Move the robot
distance = 1.0;     % Distance to move in meters
angle = pi/4;       % Angle to rotate in radians

% Update robot position and orientation
robotX = robotX + distance * cos(robotTheta);
robotY = robotY + distance * sin(robotTheta);
robotTheta = robotTheta + angle;

% Check if the robot has reached a specific target location
targetX = 2;        % Target X position in meters
targetY = 3;        % Target Y position in meters
distanceToTarget = sqrt((robotX - targetX)^2 + (robotY - targetY)^2);

% Perform actions based on the distance to the target
if distanceToTarget <= 0.1
    disp('Target reached!');   % Display a message when the target is reached
else
    disp(['Distance to target: ' num2str(distanceToTarget) ' meters']);   % Display the distance to the target
end

% Display robot information
disp('RoboMatic - Robot Information:');
disp(['Position (X, Y): (' num2str(robotX) ', ' num2str(robotY) ')']);
disp(['Orientation: ' num2str(rad2deg(robotTheta)) ' degrees']);

% Additional features:

% Generate a plot of the robot's path
figure;
plot(robotX, robotY, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
hold on;
plot(targetX, targetY, 'gx', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('X (m)');
ylabel('Y (m)');
title('RoboMatic - Robot Path');
legend('Robot', 'Target');
grid on;
axis equal;

% Calculate and display the traveled distance
traveledDistance = sqrt((robotX - 0)^2 + (robotY - 0)^2);
disp(['Traveled Distance: ' num2str(traveledDistance) ' meters']);

% Calculate and display the robot's velocity
timeElapsed = 5;    % Time elapsed in seconds
velocity = traveledDistance / timeElapsed;
disp(['Robot Velocity: ' num2str(velocity) ' m/s']);

% Perform obstacle detection
obstacleX = [1.5, 2.2, 0.8];    % X positions of obstacles
obstacleY = [0.5, 1.8, 2.5];    % Y positions of obstacles

for i = 1:length(obstacleX)
    obstacleDistance = sqrt((robotX - obstacleX(i))^2 + (robotY - obstacleY(i))^2);
    
    if obstacleDistance <= 0.2
        disp(['Obstacle detected at (' num2str(obstacleX(i)) ', ' num2str(obstacleY(i)) ')']);
    end
end
