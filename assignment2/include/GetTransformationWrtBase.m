function [bTi] = GetTransformationWrtBase(biTei, linkNumber)
%%% GetTransformatioWrtBase function
% inputs :
% biTei vector of matrices containing the transformation matrices from link i to link i +1 for the current joint position q.
% The size of biTei is equal to (4,4,numberOfLinks)
% linkNumber for which computing the transformation matrix
% outputs
% bTi : transformation matrix from the manipulator base to the ith joint in
% the configuration identified by biTei.

% Initialize the transformation matrix as an identity matrix
bTi = eye(4);

% Multiply the transformation matrices of the links from the base to the specified link
for i = 1:linkNumber
    bTi = bTi * biTei(:,:,i);
end

end