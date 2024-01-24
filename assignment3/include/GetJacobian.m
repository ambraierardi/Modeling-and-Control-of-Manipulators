function J = GetJacobian(biTei,jointType,bTe)
%% GetJacobian function
% Function returning the end effector jacobian for a manipulator which current
% configuration is described by biTei.
%
% Inputs:
% - biTei: vector of matrices containing the transformation matrices from
% joint i-1 to joint i for the current configuration.
% - bTe: current transformation matrix from base to the end effector.
% - jointType: vector identifying the joint type, 0 for revolute, 1 for
% prismatic
%
% Output:
% - J: end-effector jacobian matrix
numberOfLinks=length(jointType);
bTi = zeros(4,4,numberOfLinks); 
for i =1:numberOfLinks
    bTi(:,:,i)= GetTransformationWrtBase(biTei, i);
end
if nargin<3 %controlla nargin
    bTe=bTi(:,:,end);
    %JOINT TYPE==0 REVOLUTE, 1 PRISMATIC
    Jan0=zeros(3,max(size(jointType)));
    Jln0=zeros(3,max(size(jointType)));
    for i=1:max(size(jointType))
        if jointType(i)==0
            Jan0(:,i)=bTi(1:3,3,i);
            rin=(bTe(1:3,4)-bTi(1:3,4,i));
            Jln0(:,i)=cross(bTi(1:3,3,i),rin);
        elseif jointType(i)==1
            Jan0(:,i)=zeros(3,1);
            Jln0(:,i)=bTi(1:3,3,i);
        end
    end
    J=[Jan0;Jln0];
end
end