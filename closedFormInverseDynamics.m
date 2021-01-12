%{
Attempting closed form dynamics with the Alpha arm. 
Last modified by Hannah Kolano 1/12/2021

%}

function taulist = closedFormInverseDynamics(dof, thetalist, dthetalist, ddthetalist, Ftip, g)
    % Get kinematic and dynamic values
    [a_joint_frames, a_link_frames, MlistForward, MlistBackward, Glist, Slist, Alist] = urdfConstructionAlpha();
    alphaArm = alphaSetup();
    
    Qspace0 = [0; 0; 0; 0; 0];
    b_jacob = alphaArm.jacobe(Qspace0)
    
   % Find mass matrix
   
end