function [q] = KinematicSimulation(q, q_dot, ts, q_min, q_max)
%% Kinematic Simulation function
%
% Inputs
% - q current robot configuration
% - q_dot joints velocity
% - ts sample time
% - q_min lower joints bound
% - q_max upper joints bound
%
% Outputs
% - q new joint configuration


    % Updating q
    q=q+q_dot*ts;
    % Saturating the joint positions 
    if q<q_min
        q=q_min;
    elseif q>q_max
        q=q_max;
    end
end