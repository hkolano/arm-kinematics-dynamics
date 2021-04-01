%{
Set up the dynamics of the Seabotix vehicle
Last modified by Hannah Kolano 4/1/2021
%}

syms u v w p q r
syms udot vdot wdot pdot qdot rdot
syms x y z phi theta psi
syms u0 u1 u2 u3 u4 u5
global V Vdot eta thrustforces
V = [u v w p q r].';  % V
Vdot = [udot vdot wdot pdot qdot rdot].'; % Vdot
eta = [x y z phi theta psi].'; % wrt earth
thrustforces = [u0 u1 u2 u3 u4 u5].';

close all


%% ------------            CONSTANTS                ---------------------
% ------------------------------------------------------------------------

% ----- M_RB Mass of the Rigid Body -----
% Assume body fixed frame at estimated COM (0.06m above geometric center)
% MMoI matrix assumes vehicle density = water density
m = 22.2; % original vehicle mass
Ixx = 0.75; Iyy = 1.4; Izz = 1.6; % from SW model
M_RB = diag([m m m Ixx Iyy Izz]);

% ----- M_A Added Mass -----
% Scaled values from REXROV paper (see tab of google sheet)
M_A = [ 8.1     -0.07   -1.07   0.09    -1.71   -0.08;
       -0.07    12.69   0.53    4.25    -0.06   0.65;
       -1.07    0.53    38.02   0.06    -4.01   0.11;
       0.09     4.25    0.06    5.56    -0.10   0.22;
       -1.71    -0.06   -4.01   -0.10   8.75    -0.01;
       -0.08    0.65    0.11    0.22    -0.01   2.33];
% Diagonals of matrix X_ud, Y_vd, Z_wd...
am = diag(M_A);
M_Adiag = diag(am);

% ----- TCM Thruster Control Matrix -----
% Each thruster is a column
TCM = [0        0       0.7071  0.7071  -0.7071 -0.7071;
       -0.5     0.5     -0.7071 0.7071  -0.7071 0.7071;
       -0.866   -0.866  0       0       0       0;
       0.0011   -0.0011 -0.0495 0.0495  -0.0495 0.0495;
       0        0       0.0495  0.0495  -0.0495 -0.0495;
       0        0       -.2506  0.2506  0.2506  -0.2506];
   
% ----- Constants for changing terms -----
% DAMPING
% Nonlinear damping from drag, linear damping scaled from REXROV
lin_damp = [3.44 4.61 52.92 4.55 8.02 2.71];
nonlin_damp = [34.40 65.86 132.29 11.37 20.04 13.54];

% RESTORING FORCES
% Assume distance between R_B and R_G is 0.05m
z_b = 0.05;
B = m*9.81; % Buoyancy force (assume neutrally buoyant)


%% ------------          Changing Terms                  --------------
%-------------------------------------------------------------------------

% ----- C_RB Coriolis Matrix for the Rigid Body ----- 
% Fossen 1994, eqns 2.99 and 2.102
C_RB = [0       0       0       0       m*w     -m*v;
        0       0       0       -m*w    0       m*u;
        0       0       0       m*v     -m*u    0;
        0       m*w     -m*v    0       Izz*r   -Iyy*q;
        -m*w    0       m*u     -Izz*r  0       Ixx*p;
        m*v     -m*u    0       Iyy*q   -Ixx*p  0];
    
% ----- C_A Coriolis Matrix for the Added Mass ----- 
% Antonelli 2006, p27
C_A = [0           0           0           0           am(3)*w     -am(2)*v;
        0           0           0           -am(3)*w    0           am(1)*u;
        0           0           0           am(2)*v     -am(1)*u    0;
        0           am(3)*w     -am(2)*v    0           am(6)*r     -am(5)*q;
        -am(3)*w    0           am(1)*u     -am(6)*r    0           am(4)*p;
        am(2)*v     -am(1)*u    0           am(5)*q     -am(4)*p    0];

% ----- D Damping -----
% ~ Three degrees of estimation ~
damp_diags = lin_damp+nonlin_damp*abs(V);
D = diag(damp_diags);

% ----- G Restoring Forces -----
Gn = [0; 0; 0; -z_b*B*cos(eta(5))*sin(eta(4)); -z_b*B*sin(eta(5)); 0];

% ----- Jacobians between Body Frame and Earth Frame
J2_eta = [1     sin(eta(4))*tan(eta(5))     cos(eta(4))*tan(eta(5));
        0       cos(eta(4))                -sin(eta(4)); 
        0       sin(eta(4))/cos(eta(5))     cos(eta(4))/cos(eta(5))];
J1_eta = [  cos(eta(6))*cos(eta(5))     -sin(eta(6))*cos(eta(4))+cos(eta(6))*sin(eta(5))*sin(eta(4))    sin(eta(6))*sin(eta(4))+cos(eta(6))*cos(eta(4))*sin(eta(5));
            sin(eta(6))*cos(eta(5))     cos(eta(6))*cos(eta(4))+sin(eta(4))*sin(eta(5))*sin(eta(6))     -cos(eta(6))*sin(eta(4))+sin(eta(5))*sin(eta(6))*cos(eta(4));
            -sin(eta(5))                cos(eta(5))*sin(eta(4))                                         cos(eta(5))*cos(phi)];
J_VtoEta = [J1_eta, zeros(3);
            zeros(3), J2_eta];
        
%% -----                Equations of motion             -----
Masses = M_RB+M_Adiag;
RHS = TCM*thrustforces - ((C_RB+C_A)*V + D*V + Gn);

%% -------------        Iterate Dynamics            ----------------
% Initial conditions
% Body fixed velocity 
curr_V = [0 0 0 0 0 0].';
% Body fixed acceleration
curr_Vdot = [0 0 0 0 0 0].';
% Thruster outputs (N)
curr_u = [-10 -10 0 10 10 0].';
% Earth fixed position
curr_eta = [0 0 0 0 0 0].';
% Earth fixed velocity
curr_etadot = [0 0 0 0 0 0].';

dt = 0.05;  % time step
T = 0;      % initial time
last_time = .5; % final time
all_times = T:dt:last_time;
n = 1;  % iterator

% Storage for body fixed velocities and earth fixed positions
trajectory_Vels = zeros(6, length(all_times));
trajectory_Vels(:,1) = curr_V;
earthfixed_positions = zeros(6, length(all_times));
earthfixed_positions(:,1) = curr_eta;

% Iterate the dynamics forward from initial to final time
while n < length(all_times)
    [new_V, new_eta] = step_dynamics_forward(curr_V, curr_eta, curr_u, RHS, Masses, J_VtoEta, dt);
    trajectory_Vels(:,n+1) = new_V;
    earthfixed_positions(:,n+1) = new_eta;
    curr_V = new_V;
    curr_eta = new_eta;
    n = n+1;
end

%% ------------             Plotting                ----------------
figure
hold on
for direction = 1:6
    plot(all_times, trajectory_Vels(direction,:));
end
title('Body Fixed Velocities')
xlabel('Time (s)')
ylabel('Velocity')
legend('u', 'v', 'w', 'p', 'q', 'r')

% Take out these 3 lines if not using Peter Corke TB
End_Orientation = rpy2r(earthfixed_positions(4:6,n).');
End_Position = earthfixed_positions(1:3,n);
final_pose = SE3(End_Orientation, End_Position);

figure
axis equal
plot3(earthfixed_positions(1,:), earthfixed_positions(2,:), ...
    earthfixed_positions(3,:), 'r-o');
grid on
title('Body Fixed Frame Position in Earth Fixed Frame')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')

% Take out these 3 lines if not using Peter Corke TB
hold on
trplot(SE3(), 'length', 0.005, 'color', 'g')
 tranimate(SE3(), final_pose, 'length', 0.005)

%% ---------------            FUNCTIONS             ----------------------
% ------------------------------------------------------------------------

function [new_V, new_eta] = step_dynamics_forward(curr_V, curr_eta, curr_u, RHS, Masses, J_VtoEta, dt)
    global V eta thrustforces
    % substitute current state values into equations
    numeric_RHS = subs(RHS, [V, eta, thrustforces], [curr_V, curr_eta, curr_u]);
    numeric_J_VtoEta = subs(J_VtoEta, eta, curr_eta);
    eta_dot = numeric_J_VtoEta*curr_V;
    
    % Find acceleration of the vehicle
    new_acc = Masses\numeric_RHS;

    % Euler forward integration
    new_V = curr_V + new_acc*dt;
    new_eta = curr_eta + eta_dot*dt;
end


   
