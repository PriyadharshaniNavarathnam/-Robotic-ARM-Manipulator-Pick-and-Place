% Define the DH parameters for the ABB CRB 1300-11/0.9 (approximate values)
L(1) = Link('d', 544, 'a', 50, 'alpha', -pi/2);  % Joint 1
L(2) = Link('d', 0, 'a', 425, 'alpha', 0);      % Joint 2
L(3) = Link('d', 0, 'a', 40, 'alpha', -pi/2);   % Joint 3
L(4) = Link('d', 425, 'a', 0, 'alpha', pi/2);   % Joint 4
L(5) = Link('d', 0, 'a', 0, 'alpha', -pi/2);    % Joint 5
L(6) = Link('d', 210.904, 'a', 0, 'alpha', 0);  % Joint 6

% Create the robot model
CRB1300 = SerialLink(L, 'name', 'ABB CRB 1300-11/0.9');

% Define target end-effector positions and orientations
targets = {
    transl(0, 450, 308.096) * trotz(-pi/2) * troty(0) * trotx(pi);  % p 1
    transl(0, 450, 100) * trotx(pi) * troty(0) * trotz(-pi/2);      % ATTACH1
    transl(450.004, 0, 459.998) * trotx(pi) * troty(0) * trotz(-pi);  % p 2
    transl(450.004, 0, 300) * trotx(pi) * troty(0) * trotz(-pi);      % DETACH1
};

% Define corresponding target names
target_names = {'P1', 'ATTACH1', 'P2', 'DETACH1'};

% Initialize array to store the end-effector positions
end_effector_positions = [];

% Solve inverse kinematics for each target
for i = 1:length(targets)
    % Perform inverse kinematics
    q_solution = CRB1300.ikine(targets{i}, 'mask', [1 1 1 0 0 0]);

    % Check if a valid solution is found
    if isempty(q_solution)
        warning(['No valid IK solution found for ', target_names{i}]);
        continue;
    end

    % Plot the robot in the solution pose and display joint angles
    plot_robot(CRB1300, q_solution, target_names{i});

    % Store the end-effector position for 3D trajectory plotting
    T_computed = CRB1300.fkine(q_solution);
    end_effector_positions = [end_effector_positions; T_computed.t'];

    % Display the computed end-effector position
    disp(['Computed End-Effector Position (x, y, z) for ', target_names{i}, ':']);
    disp(T_computed.t');

    % Pause for 2 seconds to visualize
    pause(2);
end

% Plot the 3D trajectory of the end effector
figure;
plot3(end_effector_positions(:, 1), end_effector_positions(:, 2), end_effector_positions(:, 3), 'b-o', 'LineWidth', 1.5);
grid on;
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('3D Trajectory of the ABB CRB 1300-11/0.9 End-Effector');
axis([-1000 1000 -1000 1000 0 1000]);
view(3);  % Set view to 3D

% Function to plot robot and display joint angles
function plot_robot(robot, q_solution, target_name)
    % Convert joint angles to degrees
    q_deg = rad2deg(q_solution);
    
    % Display the joint angles in degrees
    disp(['Joint Angles (degrees) for ', target_name, ':']);
    disp(q_deg);
    
    % Plot the robot in the specified pose
    robot.plot(q_solution);
    
    % Set axis limits dynamically based on the robot's workspace
    axis([-1000 1000 -1000 1000 0 1000]); 
    title(['ABB CRB 1300-11/0.9 Robot Arm - ', target_name]);
end
