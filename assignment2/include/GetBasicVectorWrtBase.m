function [r] = GetBasicVectorWrtBase(biTei, linkNumber)
%%% GetBasicVectorWrtBase function 
% input :
% biTei: trasnformation matrix in between frame i and frame j 
% linkNumber: link number 
% output
% r : basic vector from frame i to the robot base frame <0>

% Compute the transformation matrix from the base to the specified link
bTi = GetTransformationWrtBase(biTei, linkNumber);

% Extract the translation part of the transformation matrix
r = bTi(1:3, 4);

end