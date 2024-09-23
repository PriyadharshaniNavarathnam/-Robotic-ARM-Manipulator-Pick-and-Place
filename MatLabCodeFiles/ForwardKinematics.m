% Define the DH parameters for the ABB CRB 1300-11/0.9 (approximate values)
L(1) = Link('d', 544, 'a', 50,     'alpha', -pi/2);  % Joint 1
L(2) = Link('d', 0,     'a', 425, 'alpha', 0);      % Joint 2
L(3) = Link('d', 0,     'a', 40, 'alpha', -pi/2);  % Joint 3
L(4) = Link('d', 425, 'a', 0,     'alpha', pi/2);   % Joint 4
L(5) = Link('d', 0,     'a', 0,     'alpha', -pi/2);  % Joint 5
L(6) = Link('d', 210.904, 'a', 0,     'alpha', 0);      % Joint 6

% Create the robot model
CRB1300 = SerialLink(L, 'name', 'ABB CRB 1300-11/0.9');

% Define joint angle adjustment
adjustment = [0 -90 0 0 0 180];

% Define joint angles for different positions
joint_angles = [
    90.000000, 31.403743, 39.248013, 0.000000, 19.348245, -0.000000;  % P1
    90.000000, 62.955232, 29.538128, -0.000000, -2.493359, 0.000000;  % ATTACH1
    90.000000, -0.350390, -13.689924, 0.000000, 104.040315, -0.000000;  % BACK
    -0.000000, -0.350000, -13.690000, 0.000000, 104.040000, -0.000000;  % POSITION2
    -0.000000, 11.687736, 36.351459, 0.000000, 41.960805, -0.000000;  % P2
    0.000000, 32.602694, 39.158000, 0.000000, 18.239306, -0.000000;  % DETACH1
    -90.000000, -0.350000, -13.690000, 0.000000, 104.040000, -0.000000;  % POSITION3
    -90.000000, 32.602694, 39.158000, 0.000000, 18.239306, -0.000000;  % P3
    -90.000000, 55.477207, 33.193827, 0.000000, 1.328966, -0.000000;  % DETACH2
];

% Define corresponding position names
position_names = {'P1', 'ATTACH1', 'BACK', 'POSITION2', 'P2', 'DETACH1', 'POSITION3', 'P3', 'DETACH2'};

% Plot and display each position with a pause
for i = 1:size(joint_angles, 1)
    plot_robot(CRB1300, joint_angles(i, :), position_names{i}, adjustment);
    pause(2);  % Pause for 2 seconds between each plot
end

% Function to plot robot and display end-effector position
function plot_robot(robot, joint_angles, position_name, adjustment)
    % Adjust joint angles
    q = deg2rad(joint_angles + adjustment);  
    % Compute forward kinematics
    P = robot.fkine(q);  
    % Display the translation part of the transformation matrix
    disp(['End-Effector Position (x, y, z) for ', position_name, ':']);
    disp(P.t');  
    % Plot the robot in the specified pose
    robot.plot(q);
    % Set axis limits dynamically based on the robot's workspace
    axis([-1000 1000 -1000 1000 0 1000]); 
    title(['ABB CRB 1300-11/0.9 Robot Arm - ', position_name]);
end
