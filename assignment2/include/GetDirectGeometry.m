function [iTj_q] = GetDirectGeometry(q, iTj, JointType)
%%% GetDirectGeometryFunction

% Inputs: 
% q : links current position ; 
% iTj : vector of matrices containing the transformation matrices from
% joint i to joint j
% JointType: vector of size numberOfLinks identiying the joint type, 0 for revolute, 1 for
% prismatic.

% Outputs :
% iTj_q vector of matrices containing the transformation matrices from joint i to joint j for the input q. 
% The size of iTj is equal to (4,4,numberOfLinks)
[d1,d2,d3]=size(iTj);
iTj_q=zeros(d1,d2,d3);
numberOfLinks=d3;
for i = 1:1:numberOfLinks
    iTj_q(:,:,i) = DirectGeometry(q(i),iTj(:,:,i), JointType(i));
end

end