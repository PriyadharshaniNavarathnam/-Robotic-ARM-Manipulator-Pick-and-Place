% Define DH parameters for the ABB CRB 1300-11/0.9 (approximate values)
L(1) = Link('d', 544, 'a', 50,     'alpha', -pi/2);  % Joint 1
L(2) = Link('d', 0,     'a', 425, 'alpha', 0);      % Joint 2
L(3) = Link('d', 0,     'a', 40, 'alpha', -pi/2);  % Joint 3
L(4) = Link('d', 425, 'a', 0,     'alpha', pi/2);   % Joint 4
L(5) = Link('d', 0,     'a', 0,     'alpha', -pi/2);  % Joint 5
L(6) = Link('d', 210.904, 'a', 0,     'alpha', 0);      % Joint 6

% Create the robot model
CRB1300 = SerialLink(L, 'name', 'ABB CRB 1300-11/0.9');

% Define the joint angles (in degrees)
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

% Convert joint angles to radians
joint_angles_rad = joint_angles * pi / 180;

% Number of steps for the animation
steps = 10;

% Create a figure for the animation
figure;
axis([-1000 1000 -1000 1000 0 1000]); % Adjust these limits according to your setup
grid on;
title('ABB CRB 1300-11/0.9 Robot Arm Animation');

% Animate the robot through the provided joint angles
for i = 1:size(joint_angles_rad, 1)
    % Update the robot configuration
    CRB1300.plot(joint_angles_rad(i, :));
    pause(1); % Adjust pause to control the speed of the animation
end
