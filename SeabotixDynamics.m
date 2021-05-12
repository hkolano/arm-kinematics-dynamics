%{
Set up the dynamics of the Seabotix vehicle
Maintained by Hannah Kolano kolanoh@oregonstate.edu
Last modified by Hannah Kolano 5/12/21
%}

close all


%% ------------            CONSTANTS                ---------------------

% ----- M_RB Mass of the Rigid Body -----
% Assume body fixed frame at estimated COM (0.06m above geometric center)
% MMoI matrix assumes vehicle density = water density
params.m = 22.2; % original vehicle mass
params.Ixx = 0.75; params.Iyy = 1.4; params.Izz = 1.6; % from SW model
mats.M_RB = diag([params.m params.m params.m params.Ixx params.Iyy params.Izz]);

% ----- M_A Added Mass -----
% Scaled values from REXROV paper (see tab of google sheet)
mats.M_A = [ 8.1     -0.07   -1.07   0.09    -1.71   -0.08;
       -0.07    12.69   0.53    4.25    -0.06   0.65;
       -1.07    0.53    38.02   0.06    -4.01   0.11;
       0.09     4.25    0.06    5.56    -0.10   0.22;
       -1.71    -0.06   -4.01   -0.10   8.75    -0.01;
       -0.08    0.65    0.11    0.22    -0.01   2.33];
% Diagonals of matrix X_ud, Y_vd, Z_wd...
params.am = diag(mats.M_A);
mats.M_Adiag = diag(params.am);
mats.M = mats.M_RB+mats.M_Adiag;

% ----- TCM Thruster Control Matrix -----
% Each thruster is a column
mats.TCM = [0        0       0.7071  0.7071  -0.7071 -0.7071;
       -0.5     0.5     -0.7071 0.7071  -0.7071 0.7071;
       -0.866   -0.866  0       0       0       0;
       0.8461   -0.8461 -0.0566 0.0566  -0.0566 0.0566;
       0        0       0.0566  0.0566  -0.0566 -0.0566;
       0        0       -.2506  0.2506  0.2506  -0.2506];
   
% ----- Constants for changing terms -----
% DAMPING
% Nonlinear damping from drag, linear damping scaled from REXROV
params.lin_damp = [3.44 4.61 52.92 4.55 8.02 2.71];
params.nonlin_damp = [34.40 65.86 132.29 11.37 20.04 13.54];

% RESTORING FORCES
% Assume distance between R_B and R_G is 0.05m
params.z_b = 0.05;
params.Buoy = params.m*9.81; % Buoyancy force (assume neutrally buoyant)

%% -------------        Iterate Dynamics            ----------------
% Initial conditions
% Body fixed velocity 
curr_V = [0 0 0 0 0 0].';
% Body fixed acceleration
curr_Vdot = [0 0 0 0 0 0].';
% Thruster outputs (N)
curr_u = [-10 -10 0 -20 -20 0].';
% Earth fixed position
curr_eta = [0 0 0 0 0 0].';
% Earth fixed velocity
curr_etadot = [0 0 0 0 0 0].';

dt = 0.05;  % time step
T = 0;      % initial time
last_time = 1.00; % final time
all_times = T:dt:last_time;
n = 1;  % iterator

% Storage for body fixed velocities and earth fixed positions
trajectory_Vels = zeros(6, length(all_times));
trajectory_Vels(:,1) = curr_V;
earthfixed_positions = zeros(6, length(all_times));
earthfixed_positions(:,1) = curr_eta;

% Iterate the dynamics forward from initial to final time
while n < length(all_times)
    fprintf("%d/%d\n",n, length(all_times))

    [new_V, new_eta] = step_dynamics_forward(curr_V, curr_eta, curr_u, dt, params, mats);
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

figure
hold on
for direction = 1:6
    plot(all_times, earthfixed_positions(direction,:));
end
title('Earth Fixed Positions')
xlabel('Time (s)')
ylabel('Position (m, rad)')
legend('x', 'y', 'z', 'roll', 'pitch', 'yaw')

% Take out these 3 lines if not using Peter Corke TB
End_Orientation = rpy2r(earthfixed_positions(4:6,n).');
End_Position = earthfixed_positions(1:3,n);
final_pose = SE3(End_Orientation, End_Position);

figure
axis([-5 5 -5 5 -5 5])
plot3(earthfixed_positions(1,:), earthfixed_positions(2,:), ...
    earthfixed_positions(3,:), 'r-o');
hold on
plot3([.005, -.002], [-.002, .005], [0, .05], 'k.')
grid on
title('Body Fixed Frame Position in Earth Fixed Frame')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')

% Take out these 3 lines if not using Peter Corke TB
trplot(SE3(), 'length', 0.005, 'color', 'g', 'axis', [-5 5 -5 5 -5 5])
 tranimate(SE3(), final_pose, 'length', 0.005, 'axis', [-5 5 -5 5 -5 5])

%% ---------------            FUNCTIONS             ----------------------
% ------------------------------------------------------------------------

function [new_V, new_eta] = step_dynamics_forward(curr_V, curr_eta, curr_u, dt, params, mats)

    % substitute current state values into EOM
    numeric_RHS = computeRHS(curr_V, curr_eta, curr_u, params, mats);
    
    % Find velocity in world frame
    numeric_J_VtoEta = computeJ_VtoEta(curr_eta);
    eta_dot = numeric_J_VtoEta*curr_V;
    
    % Find acceleration of the vehicle in body frame
    new_acc = mats.M\numeric_RHS;

    % Euler forward integration
    new_V = curr_V + new_acc*dt; % new body fixed velocity
    new_eta = curr_eta + eta_dot*dt; % new earth fixed position
end

function numeric_J_VtoEta = computeJ_VtoEta(eta)

    J2_eta = [1     sin(eta(4))*tan(eta(5))     cos(eta(4))*tan(eta(5));
            0       cos(eta(4))                -sin(eta(4)); 
            0       sin(eta(4))/cos(eta(5))     cos(eta(4))/cos(eta(5))];
    J1_eta = [  cos(eta(6))*cos(eta(5))     -sin(eta(6))*cos(eta(4))+cos(eta(6))*sin(eta(5))*sin(eta(4))    sin(eta(6))*sin(eta(4))+cos(eta(6))*cos(eta(4))*sin(eta(5));
                sin(eta(6))*cos(eta(5))     cos(eta(6))*cos(eta(4))+sin(eta(4))*sin(eta(5))*sin(eta(6))     -cos(eta(6))*sin(eta(4))+sin(eta(5))*sin(eta(6))*cos(eta(4));
                -sin(eta(5))                cos(eta(5))*sin(eta(4))                                         cos(eta(5))*cos(eta(4))];

    numeric_J_VtoEta = [J1_eta, zeros(3);
                zeros(3), J2_eta];
end


function numeric_RHS = computeRHS(curr_V, curr_eta, curr_u, params, mats)

    % was too lazy to do find and replace on the matrices below
    % can refactor this for cleanliness later
    u = curr_V(1);
    v = curr_V(2); 
    w = curr_V(3); 
    p = curr_V(4); 
    q = curr_V(5); 
    r = curr_V(6); 

    % ----- C_RB Coriolis Matrix for the Rigid Body ----- 
    % Fossen 1994, eqns 2.99 and 2.102
    C_RB = [0           0           0           0              params.m*w       -params.m*v;
            0           0           0           -params.m*w    0                params.m*u;
            0           0           0           params.m*v     -params.m*u      0;
            0           params.m*w  -params.m*v 0              params.Izz*r     -params.Iyy*q;
            -params.m*w 0           params.m*u  -params.Izz*r  0                params.Ixx*p;
            params.m*v  -params.m*u 0           params.Iyy*q   -params.Ixx*p    0];

    % ----- C_A Coriolis Matrix for the Added Mass ----- 
    % Antonelli 2006, p27
    C_A = [0                    0               0               0                   params.am(3)*w     -params.am(2)*v;
            0                   0               0               -params.am(3)*w    0                    params.am(1)*u;
            0                   0               0               params.am(2)*v     -params.am(1)*u      0;
            0                   params.am(3)*w  -params.am(2)*v 0                   params.am(6)*r     -params.am(5)*q;
            -params.am(3)*w     0               params.am(1)*u  -params.am(6)*r    0                    params.am(4)*p;
            params.am(2)*v      -params.am(1)*u 0               params.am(5)*q     -params.am(4)*p      0];

    damp_diags = params.lin_damp+params.nonlin_damp*abs(curr_V);
    D = diag(damp_diags);
    Gn = [0; 0; 0; -params.z_b*params.Buoy*cos(curr_eta(5))*sin(curr_eta(4)); -params.z_b*params.Buoy*sin(curr_eta(5)); 0];

    numeric_RHS = mats.TCM*curr_u - ((C_RB+C_A)*curr_V + D*curr_V + Gn);
end

   
