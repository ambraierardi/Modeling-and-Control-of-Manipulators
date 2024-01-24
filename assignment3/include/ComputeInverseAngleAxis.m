function [theta,v] = ComputeInverseAngleAxis(R)
%EULER REPRESENTATION: Given a tensor rotation matrices this function
% should output the equivalent angle-axis representation values,
% respectively 'theta' (angle), 'v' (axis) 
% SUGGESTED FUNCTIONS
    % size()
    % eye()
    % eig()
    % find()
    % abs()
    % det()
    % NB: Enter a square, 3x3 proper-orthogonal matrix to calculate its angle
    % and axis of rotation. Error messages must be displayed if the matrix
    % does not satisfy the rotation matrix criteria.
    
    % Check matrix R to see if its size is 3x3
    [r,c]=size(R);
    if r==3 && c==3
        % Check matrix R to see if it is orthogonal
        if abs(R*transpose(R)-eye(3))<=1e-4 
            % Check matrix R to see if it is proper: det(R) = 1
            if abs(det(R)-1)<=1e-4
                % Compute the angle of rotation
                theta=acos((trace(R)-1)/2);
                % Calculate eigenvalues and eigenvectors of R
                [V,D]=eig(R);
                % Compute the axis of rotation
                a=find(abs(D-1)<=1e-4);
                v=V(:,sqrt(a));
                R_neg=ComputeAngleAxis(theta,-v);
                if abs(R_neg-R)<=1e-4
                    theta=-theta;
                end

            else
              disp('DETERMINANT OF THE INPUT MATRIX IS NOT 1')
            end
        else
             disp('NOT ORTHOGONAL INPUT MATRIX')
        end
    else
       disp('WRONG SIZE OF THE INPUT MATRIX')
    end
end

