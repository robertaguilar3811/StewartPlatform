clc
clear all
close all

%% Enter Coordinates and Euler Angles

% Enter Cartesian Coordinates
% Start
x1 = 0;
y1 = 0;
z1 = 0;

% End
x2 = 0;
y2 = 0;
z2 = 0;

%Enter Euler Angles
% Start
phi1 = 0; %pitch, about x
theta1 = 0; %roll, about y
psi1 = 0; %yaw, about z

phi1 = deg2rad(phi1);
theta1 = deg2rad(theta1);
psi1 = deg2rad(psi1);

% End
phi2 = -10; %pitch, about x
theta2 = 20; %roll, about y
psi2 = -30; %yaw, about z

phi2 = deg2rad(phi2);
theta2 = deg2rad(theta2);
psi2 = deg2rad(psi2);

%R1 = generateMatrix(R, x1, y1, z1)
%R2 = generateMatrix(R,x2, y2, z2)

%% Generate Base & Platform Coordinates

% Calculate base coordinates
th = [30 90 150 210 270 330];
th = deg2rad(th);
rho = [1 1 1 1 1 1];

[x, y] = pol2cart(th, rho);

b_1 = [x(1) y(1) 0]';
b_2 = [x(2) y(2) 0]';
b_3 = [x(3) y(3) 0]';
b_4 = [x(4) y(4) 0]';
b_5 = [x(5) y(5) 0]';
b_6 = [x(6) y(6) 0]';

B = [b_1 b_2 b_3 b_4 b_5 b_6];

% Calculate platform coordinates
th = [30 90 150 210 270 330];
th = deg2rad(th);
rho = [0.75 0.75 0.75 0.75 0.75 0.75];

[x, y] = pol2cart(th, rho);

p_1 = [x(1) y(1) 0]';
p_2 = [x(2) y(2) 0]';
p_3 = [x(3) y(3) 0]';
p_4 = [x(4) y(4) 0]';
p_5 = [x(5) y(5) 0]';
p_6 = [x(6) y(6) 0]';

P = [p_1 p_2 p_3 p_4 p_5 p_6];

%% Translation from Base to Platform

T = [0 0 1]'; %translate from B to P 1 meter along z

R1 = generateRotationMatrix(phi1, theta1, psi1);
R2 = generateRotationMatrix(phi2, theta2, psi2);

%% Generate Start & End Leg Lengths

l1 = inverseKinematics(T, R1, B, P);
l2 = inverseKinematics(T, R2, B, P);

%% Animate Stewart Platform

% Number of animation frames
nFrames = 100;

% Interpolate rotation angles
phi_anim = linspace(phi1, phi2, nFrames);
theta_anim = linspace(theta1, theta2, nFrames);
psi_anim = linspace(psi1, psi2, nFrames);

for k = 1:nFrames
    cla
    grid on
    axis equal
    xlim([-2 2]); ylim([-2 2]); zlim([0 2]);  % <-- Apply limits here
    
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(3)
    
    % Current rotation
    Rk = generateRotationMatrix(phi_anim(k), theta_anim(k), psi_anim(k));
    
    % Platform joints in global frame
    P_global = Rk * P + T;

    % === Draw Base ===
    fill3(B(1,:), B(2,:), B(3,:), 'k', 'FaceAlpha', 0.6, 'EdgeColor', 'k'); hold on;
    plot3(B(1,:), B(2,:), B(3,:), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    
    % === Draw Platform ===
    fill3(P_global(1,:), P_global(2,:), P_global(3,:), 'r', 'FaceAlpha', 0.6, 'EdgeColor', 'r');
    plot3(P_global(1,:), P_global(2,:), P_global(3,:), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

    % === Draw Legs ===
    for i = 1:6
        plot3([B(1,i), P_global(1,i)], ...
              [B(2,i), P_global(2,i)], ...
              [B(3,i), P_global(3,i)], ...
              'b', 'LineWidth', 2);
    end
    
    drawnow;

end

%% Generate Rotation Matrix

function R = generateRotationMatrix(phi, theta, psi)

    Rz = [cos(psi) -sin(psi) 0
        sin(psi) cos(psi) 0
        0 0 1];
    
    Ry = [cos(theta) 0 sin(theta)
        0 1 0
        -sin(theta) 0 cos(theta)];
    
    Rx = [1 0 0
        0 cos(phi) -sin(phi)
        0 sin(phi) cos(phi)];
    
    R = Rz*Ry*Rx;

end

%% Inverse Kinematics Solver

function l = inverseKinematics(T, R_BP, B, P)

    b_1 = B(:,1);
    b_2 = B(:,2);
    b_3 = B(:,3);
    b_4 = B(:,4);
    b_5 = B(:,5);
    b_6 = B(:,6);
    
    p_1 = P(:,1);
    p_2 = P(:,2);
    p_3 = P(:,3);
    p_4 = P(:,4);
    p_5 = P(:,5);
    p_6 = P(:,6);

    % Generates leg lengths based on T, R, B, P
    %1
    %q_1 = T + R_BP*p_1;
    l_1 = T + R_BP*p_1 - b_1;
    l_1 = norm(l_1);
    
    %2
    %q_2 = T + R_BP*p_2;
    l_2 = T + R_BP*p_2 - b_2;
    l_2 = norm(l_2);
    
    %3
    %q_3 = T + R_BP*p_3;
    l_3 = T + R_BP*p_3 - b_3;
    l_3 = norm(l_3);
    
    %4
    %q_4 = T + R_BP*p_4;
    l_4 = T + R_BP*p_4 - b_4;
    l_4 = norm(l_4);
    
    %5
    %q_5 = T + R_BP*p_5;
    l_5 = T + R_BP*p_5 - b_5;
    l_5 = norm(l_5);
    
    %6
    %q_6 = T + R_BP*p_6;
    l_6 = T + R_BP*p_6 - b_6;
    l_6 = norm(l_6);
    
    l = [l_1 l_2 l_3 l_4 l_5 l_6]';

end