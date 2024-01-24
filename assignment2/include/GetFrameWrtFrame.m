function [iTj] = GetFrameWrtFrame(linkNumber_i, linkNumber_j, biTei)
%%% GetFrameWrtFrame function 
% inputs : 
% linkNumber_i : number of ith link 
% linkNumber_j: number of jth link 
% biTei vector of matrices containing the transformation matrices from link i to link i +1 for the current q.
% The size of biTei is equal to (4,4,numberOfLinks)
% outputs:
% iTj : transformationMatrix in between link i and link j for the
% configuration described in biTei

% Initialize the transformation matrix as an identity matrix
iTj = eye(4);

% Multiply the transformation matrices of the links between link i and link j
if (linkNumber_i < linkNumber_j)
    for i = linkNumber_i:linkNumber_j
        iTj = iTj * biTei(:,:,i);
    end
elseif (linkNumber_i > linkNumber_j)
     for i = linkNumber_j:linkNumber_i
        R_t = biTei(1:3,1:3,i)';
        last_col = biTei(1:3,4,i);
        iTj(1:3,1:3) = R_t;
        iTj(1:3,4) = -R_t * last_col;
        iTj(4,:) = [0,0,0,1];
     end
end

end