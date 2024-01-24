function iTj_q = DirectGeometry(qi, iTj, JointType)
% DirectGeometry Function 
% inputs: 
% qi : current joint position;
% iTj is the constant transformation between the base of the link <i>
% and its follower frame <j>; 
% jointType :0 for revolute, 1 for prismatic

% output :
% iTj_q : transformation between the base of the joint <i> and its follower frame taking 
% into account the actual rotation/traslation of the joint
iTj_q=zeros(size(iTj));
Rz=[cos(qi) -sin(qi) 0; sin(qi) cos(qi) 0; 0 0 1];
if JointType == 0 % rotational
    iTj_q(1:3,4)=iTj(1:3,4);
    iTj_q(1:3,1:3)=iTj(1:3,1:3)*Rz;
elseif JointType == 1 % prismatic
    iTj_q(1:3,1:3)=iTj(1:3,1:3);
    iTj_q(1:3,4)=iTj(1:3,4)+qi*iTj(1:3,3);
end
iTj_q(4,4)=1;
end