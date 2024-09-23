clc;
clear;
close all;

% Generate a Robolink object RDK. This object interfaces with RoboDK.
RDK = Robolink;

% Define the path to your specific RDK file as a character array
rdkFilePath = 'E:\G27\G27_Task01.rdk'; % Use single quotes for character array

% Load your RDK file
station = RDK.AddFile(rdkFilePath);

% Check if the station was successfully loaded
if ~station.Valid()
    RDK.ShowMessage(sprintf('Failed to load the RDK file at path:<br>%s.', rdkFilePath));
    return
end

% Display a list of all items
fprintf('Available items in the station:\n');
disp(RDK.ItemList(-1, 1));

% Retrieve the robot by its name or type
robotName = 'ABB CRB 1300-11/0.9'; 
robot = RDK.Item(robotName, RDK.ITEM_TYPE_ROBOT);

% Check if the robot item was found
if ~robot.Valid()
    fprintf('Robot "%s" not found in the station.\n', robotName);
    return
end

% Define the target points (in mm)
points = [
    374  0    630;
    0    450  580;
    0    450  285;
    0    450  580;
    400  0    580;
    400  0    461;
    400  0    580;
    100  0    580;
    400  0    580;
    400  0    461;
    400  0    580;
    0    -450 580;
    0    -450 281;
    0    -450 580;
    374  0    630
];

% Number of samples between points
samples = 10;

% Initialize arrays for trajectory points
x = [];
y = [];
z = [];

% Generate trajectory points
for i = 1:size(points, 1)-1
    % Interpolate points between the current and next target
    x_temp = linspace(points(i, 1), points(i+1, 1), samples);
    y_temp = linspace(points(i, 2), points(i+1, 2), samples);
    z_temp = linspace(points(i, 3), points(i+1, 3), samples);
    
    % Append points
    x = [x; x_temp'];
    y = [y; y_temp'];
    z = [z; z_temp'];
end

% Initialize the transformation matrices
T = zeros(4, 4, length(x));

% Fixed orientation (e.g., identity matrix for no rotation)
R_fixed = eye(3);

% Create transformation matrices for each point with fixed orientation
for i = 1:length(x)
    T(:,:,i) = [R_fixed, [x(i); y(i); z(i)];
                0, 0, 0, 1];
end

% Initialize an array for robot joints
q = zeros(length(x), 4);

% Prepare the 3D plot
figure;
hold on;
grid on;
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('4-DOF Robot Arm Trajectory in 3D Space');
axis equal;

% Plot the trajectory points
plot3(x, y, z, 'r--o', 'LineWidth', 1.5, 'MarkerSize', 5);

% Set plot limits for better visibility
xlim([-500 500]);
ylim([-500 500]);
zlim([0 700]);

% Loop through each point to calculate joint configurations and set robot pose
for i = 1:length(x)
    % Calculate the joints for each transformation matrix
    joints = robot.SolveIK(T(:,:,i));
    
    if ~any(isnan(joints)) && ~isempty(joints)
        q(i, :) = joints; % Store joint configuration
        robot.setJoints(joints); % Set the robot joints in RoboDK
        
        % Extract the robot arm link positions for visualization
        [robot_pos, link_pos] = robot.JointsPosition();

        % Plot the robot arm movement
        for j = 1:size(link_pos, 2) - 1
            plot3([link_pos(1, j), link_pos(1, j+1)], ...
                  [link_pos(2, j), link_pos(2, j+1)], ...
                  [link_pos(3, j), link_pos(3, j+1)], 'b-', 'LineWidth', 2);
        end
        
        % Pause to animate the movement
        pause(0.1);
    else
        fprintf('No valid solution found for pose %d. Skipping this point.\n', i);
    end
end

% Final plot adjustments
view(3);