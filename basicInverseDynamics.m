
function taulist = basicInverseDynamics(dof, thetalist, dthetalist, ddthetalist, Ftip, g)
    % Get kinematic and dynamic values
    [a_joint_frames, a_link_frames, MlistForward, MlistBackward, Glist, Slist, Alist] = urdfConstructionAlpha();
    
    % Twist of base in world frame 
    V0 = [0; 0; 0; 0; 0; 0];
    % Acceleration of base in world frame (gravity)
    V0_dot = [0; 0; 0; 0; 0; -9.81];
    
    % Forward iterations
    for i = 1:dof
        % Relative frame configuration based on current angle
        i
        A_i = Alist(:, i);
        theta_i = thetalist(i);
        M_i_iminus1 = MlistBackward(:,:,i);
        % Product of exponentials
        T_i_iminus1 = FKinSpace(M_i_iminus1, [-A_i], [theta_i]);
    end
end