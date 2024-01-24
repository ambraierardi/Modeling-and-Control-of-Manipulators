function R = ComputeAngleAxis(theta,v)
%Implement here the Rodrigues formula
    skew_matrix=[0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
    skew_matrix_squared=v*transpose(v)-eye(3);
    R=eye(3)+sin(theta)*skew_matrix+(1-cos(theta))*skew_matrix_squared;
end
