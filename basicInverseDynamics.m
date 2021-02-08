
function ActuatorTorques = basicInverseDynamics(dof, thetalist, dthetalist, ddthetalist, Ftip)
    % Get kinematic and dynamic values
    [~, ~, MlistForward, MlistBackward, Slist, Alist, Glist] = AlphaKinematics();
    
    % Twist of base in world frame 
    V0 = [0; 0; 0; 0; 0; 0];
    % Acceleration of base in world frame (gravity)
    V0_dot = [0; 0; 0; 0; 0; -9.81];
    
    % LinkTwists{1} = base, LinkTwists{2} = link1, etc
    LinkTwists = {};
    LinkTwists{1} = V0;
    DeltaLinkTwists = {};
    DeltaLinkTwists{1} = V0_dot;
    
    T_frames_from_tip{6} = MlistBackward(:,:,6);
    ForcesThrough{6} = Ftip;
    ActuatorTorques = [0; 0; 0; 0; 0];
    
    % Forward iterations
    for i = 1:dof
        % TO-DO: condense these lines 
        A_i = Alist(:, i);
        theta_i = thetalist(i);
        dtheta_i = dthetalist(i);
        ddtheta_i = ddthetalist(i);
        M_i_iminus1 = MlistBackward(:,:,i);
        
        % Relative frame configuration based on current angle
        % Product of exponentials
        T_i_iminus1 = FKinSpace(M_i_iminus1, [-A_i], [theta_i]);
        T_frames_from_tip{i} = T_i_iminus1;
        
        % Calculate twist of link
        V_i = Adjoint(T_i_iminus1)*LinkTwists{i} + A_i*dthetalist(i);
        LinkTwists{i+1} = V_i;
        
        % Calculate acceleration (dtwist) of link
        Vdot_i = Adjoint(T_i_iminus1)*DeltaLinkTwists{i} + ad(V_i)*A_i*dtheta_i + A_i*ddtheta_i;
        DeltaLinkTwists{i+1} = Vdot_i;
    end
    
    for j = dof:-1:1
        G_j = Glist(:,:,j);
        F_jplus1 = ForcesThrough{j+1};
        V_j = LinkTwists{j+1};
        Vdot_j = DeltaLinkTwists{j+1};
        T_jplus1_j = T_frames_from_tip{j+1};
        A_j = Alist(:, j);
        
        % Calculate wrench transmitted through joint i to link {i}
        F_j = transpose(Adjoint(T_jplus1_j))*F_jplus1 + G_j*Vdot_j - transpose(ad(V_j))*G_j*V_j;
        ForcesThrough{j} = F_j;
        
        % Find torque of actuator
        tau_j = transpose(F_j)*A_j;
        ActuatorTorques(j) = tau_j;
    end
end