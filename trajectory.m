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

% Define joint angle adjustment
adjustment = [0 -90 0 0 0 180];

% Define joint angles for different positions
joint_angles = [
    90.000000, 45.958063, 0.408965, 0.000000, 44.213781, -0.000000;  % S1PD
    90.000000, 34.442298, -33.446424, 0.000000, 89.584935, -0.000000;  % S1M1
   90.000000, -13.954059, -63.899943, -0.000000, 32.854001, 0.000000;  % S1M2
    0.000000, -13.950000, -63.900000, -0.000000, 32.850000, -0.000000;  % S2M1
    0.749740, 32.928022, -31.843716, -0.580862, 88.923349, 0.760693;  % S2M2
    0.647886, 31.376326, -14.897182, -0.605628, 73.528294, 0.819646;  % S2PD
    -90.000000, -13.950000, -63.900000, -0.000000, 32.850000, -0.000000;  % S3M1
    -90.000000, 37.105744, -49.130470, 0.000000, 101.474917, -0.000000;  % S3M2
    -90.000000, 39.305162, 1.085147, 0.000000, 49.028883, -0.000000;  % S3PD
];


% Define corresponding position names
position_names = {'S1PD', 'S1M1', 'S1M2', 'S2M1', 'S2M2', 'S2PD', 'S3M1', 'S3M2', 'S3PD'};
% Calculate joint velocities
joint_velocities = zeros(size(joint_angles, 1) - 1, size(joint_angles, 2));  % Initialize matrix for velocities
for i = 1:size(joint_angles, 1) - 1
    joint_velocities(i, :) = joint_angles(i+1, :) - joint_angles(i, :);
end

% Plot and display each position with a pause
for i = 1:size(joint_angles, 1)
    plot_robot(CRB1300, joint_angles(i, :), position_names{i}, adjustment);
    pause(2);  % Pause for 2 seconds between each plot
end

% Plot velocity diagram
figure;
num_positions = size(joint_angles, 1) - 1;
t = 1:num_positions;  % Time or position index for velocities

% Plot for each joint in separate subplots
for j = 1:size(joint_velocities, 2)
    subplot(3, 2, j);  % Adjust the subplot grid if needed
    plot(t, joint_velocities(:, j), 'o-', 'LineWidth', 1.5, 'MarkerSize', 6);
    xlabel('Position Index');
    ylabel('Velocity (degrees/s)');
    title(['Joint ', num2str(j), ' Velocity']);
    grid on;
    legend(['Joint ', num2str(j)], 'Location', 'best');
end

sgtitle('Joint Velocities Over Different Positions');

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
    title(['ABB IRB 120-3/0.6 Robot Arm - ', position_name]);
end