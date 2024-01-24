function [rot_matrix] = quatToRot(q0,q1,q2,q3)
% quatToRot convert a quaternion into a rotation matrix
    %Covert a quaternion into a full three-dimensional rotation matrix.
    %mu=q0 and epsilon=[q1;q2;q3]
    
    %Input
    %:param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    %Output
    %return: A 3x3 element matrix representing the full 3D rotation matrix. 
    epsilon=[q1;q2;q3];
    %3x3 rotation matrix
    epsilon_skew=[0 -epsilon(3) epsilon(2); epsilon(3) 0 -epsilon(1); -epsilon(2) epsilon(1) 0];
    rot_matrix=eye(3)+2*q0*epsilon_skew+2*epsilon_skew*epsilon_skew;
end