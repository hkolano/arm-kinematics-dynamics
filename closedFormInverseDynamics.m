%{
Attempting closed form dynamics with the Alpha arm. 
Last modified by Hannah Kolano 1/12/2021

%}

function [MassMatrix, Coriolis, GravityMatrix, JTFtip] = closedFormInverseDynamics(thetalist, dthetalist, ddthetalist, Ftip, g)
    % Get kinematic and dynamic values
    [~, ~, MlistForward, MlistBackward, Slist, Alist, Glist] = AlphaKinematics();
    M_home = [-1 0 0 -.3507; 0 1 0 0; 0 0 -1 0.0262; 0 0 0 1];
    
    % Joint configuration 
    Qspace = thetalist;
    J_s = JacobianSpace(Slist, thetalist);
    T_end = FKinSpace(M_home, Slist, thetalist)*MlistForward(6);
    J_b = Adjoint(TransInv(T_end))*J_s;
    JTFtip = J_b.'*Ftip;
    
%% Construct matrix of A_i, G_i
    % Twists in link frames
    A_mat = zeros(30, 5);
    % Spatial inertia matrix
    G_mat = zeros(30, 30);
    for i = 1:5
        A_mat((i*6)-5:i*6, i) = Alist(:, i);
        G_mat((i*6)-5:i*6, (i*6)-5:i*6) = Glist(:,:,i);          
    end    
    
%% Calculate VdotBase_mat 
    % Acceleration of the base link
    Vdot0 = [0; 0; 0; g]; 
    % Find T_1_0
    M_1_0 = MlistBackward(:,:,1);
    T_1_0 = FKinSpace(M_1_0, [-Alist(:,1)], [thetalist(1)]);
    VdotBase_mat = zeros(30, 1);
    VdotBase_mat(1:6) = [Adjoint(T_1_0)*Vdot0];
    
%% Start list of V (twists of links)
    % Twist of base frame {0} in {0} frame
    V_list = zeros(6,5);
    % Twist of link frame (1}
    V_list(:,1) = Adjoint(T_1_0)*[0 0 0 0 0 0].' + Alist(:,1)*dthetalist(1);
    
%% Calculate W and L Matrices; fill out V_mat
    W_mat = zeros(30,30);
    for i = 2:5
        % Get variables for this link
        A_i = Alist(:, i);
        theta_i = Qspace(i);
        dtheta_i = dthetalist(i);
        M_i_iminus1 = MlistBackward(:,:,i);
        T_i_iminus1 = FKinSpace(M_i_iminus1, [-A_i], [theta_i]);
        
        % Fill out V_list
        V_list(:,i) = Adjoint(T_i_iminus1)*V_list(:,i-1)+A_i*dtheta_i;
        % Fill out W_mat
        W_mat(i*6-5:i*6,(i-1)*6-5:(i-1)*6) = Adjoint(T_i_iminus1); 
    end
    
    L_mat = inv(eye(30)-W_mat);
    
%% Make AdV_mat
    adV_mat = zeros(30,30);
    ad_A_dtheta_mat = zeros(30,30);
    for i = 1:5
        adV_mat(i*6-5:i*6, i*6-5:i*6) = ad(V_list(:,i));
        ad_A_dtheta_mat(i*6-5:i*6, i*6-5:i*6) = ad(Alist(:,i)*dthetalist(i));
    end
    
    adV_mat;
    ad_A_dtheta_mat;
    
    
%% Closed Form Dynamics;
    MassMatrix = A_mat.'*L_mat.'*G_mat*L_mat*A_mat;
    Coriolis = -A_mat.'*L_mat.'*(G_mat*L_mat*ad_A_dtheta_mat*W_mat + adV_mat.'*G_mat)*L_mat*A_mat*dthetalist;
    GravityMatrix = A_mat.'*L_mat.'*G_mat*L_mat*VdotBase_mat;

end