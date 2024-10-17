clc
clear
import robotics.*

% Define waypoints based on provided joint angles
waypoints = [
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


% Time vector (assuming 1 second between each waypoint)
t = 0:size(waypoints, 1) - 1;

% Generate trajectory using spline interpolation
tq = 0:0.1:t(end);
trajectory = zeros(numel(tq), 6);
for i = 1:6
    trajectory(:,i) = spline(t, waypoints(:,i), tq);
end

% Calculate velocities
dt = 0.1; % Time step
velocities = diff(trajectory) / dt;

% Adjust time vector for velocity
time_velocity = tq(1:end-1);

% Plot velocity profiles
figure;
hold on;
plot(time_velocity, velocities(:,1), 'DisplayName', 'Joint 1 Velocity');
plot(time_velocity, velocities(:,2), 'DisplayName', 'Joint 2 Velocity');
plot(time_velocity, velocities(:,3), 'DisplayName', 'Joint 3 Velocity');
plot(time_velocity, velocities(:,4), 'DisplayName', 'Joint 4 Velocity');
plot(time_velocity, velocities(:,5), 'DisplayName', 'Joint 5 Velocity');
plot(time_velocity, velocities(:,6), 'DisplayName', 'Joint 6 Velocity');
title('Joint Velocity Profiles');
xlabel('Time (s)');
ylabel('Velocity (degrees/s)');
legend('show');
grid on;

% Define DH parameters for the robot
% Replace these with the actual parameters for your robot
% Example parameters (not accurate, replace with real DH parameters)
a = [0 0 0 0 0 0];
d = [0 0 0 0 0 0];
alpha = [0 0 0 0 0 0];

% Forward Kinematics Calculation
x_fk = zeros(size(tq));
y_fk = zeros(size(tq));
z_fk = zeros(size(tq));

for i = 1:length(tq)
    theta = trajectory(i, :);
    T = eye(4); % Initialize the transformation matrix
    
    % Compute the transformation matrix for each joint
    for j = 1:6
        A = [cosd(theta(j)) -sind(theta(j)) 0 a(j);
             sind(theta(j))*cosd(alpha(j)) cosd(theta(j))*cosd(alpha(j)) -sind(alpha(j)) -sind(alpha(j))*d(j);
             sind(theta(j))*sind(alpha(j)) cosd(theta(j))*sind(alpha(j)) cosd(alpha(j)) cosd(alpha(j))*d(j);
             0 0 0 1];
        T = T * A; % Update the transformation matrix
    end
    
    % Extract position from transformation matrix
    x_fk(i) = T(1, 4);
    y_fk(i) = T(2, 4);
    z_fk(i) = T(3, 4);
end

% Inverse Kinematics (Example, replace with real IK calculations)
x_inv = x_fk; % Placeholder, replace with actual inverse kinematics calculations
y_inv = y_fk; % Placeholder
z_inv = z_fk; % Placeholder

% Plot joint angles
figure;
plot(tq, trajectory(:,1), tq, trajectory(:,2), tq, trajectory(:,3), ...
     tq, trajectory(:,4), tq, trajectory(:,5), tq, trajectory(:,6));
title('Joint Angles');
xlabel('Time (s)');
ylabel('Angle (degrees)');
legend('θ1', 'θ2', 'θ3', 'θ4', 'θ5', 'θ6');
grid on;

