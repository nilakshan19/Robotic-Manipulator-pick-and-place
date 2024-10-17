clc
clear
% Define the DH parameters for the ABB IRB 120-3/0.6 
L(1) = Link('d', 290,  'a', 0,     'alpha', pi/2);  % Joint 1
L(2) = Link('d', 0,    'a', -270,  'alpha', 0);     % Joint 2
L(3) = Link('d', 0,    'a', -70,   'alpha', pi/2);  % Joint 3
L(4) = Link('d', 302,  'a', 0,     'alpha', pi/2);  % Joint 4
L(5) = Link('d', 0,    'a', 0,     'alpha', pi/2);  % Joint 5
L(6) = Link('d', 130,   'a', 0,     'alpha', 0);     % Joint 6


% Create the robot model
CRB1300 = SerialLink(L, 'name', 'ABB IRB 120-3/0.6');

% Define target end-effector positions and orientations
targets = {
    transl(0, 450.99, 235.433) * trotz(-pi/2) * troty(0) * trotx(pi);  % S1PD
    transl(450, 3.041, 429.985) * trotx(pi) * troty(0) * trotz(pi);    % S2PD
    transl(0, -448.456, 284.554) * trotx(pi) * troty(0) * trotz(pi);   % S3PD
    transl(0, 450.991, 505.433) * trotx(-pi/2) * troty(0) * trotz(pi);   % S1M1
};

% Define corresponding target names
target_names = {'S1PD', 'S2PD', 'S3PD', 'S1M1'};

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
    title(['ABB IRB 120-3/0.6 Robot Arm - ', target_name]);
end