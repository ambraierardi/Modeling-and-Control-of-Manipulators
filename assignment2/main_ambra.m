%% Modelling and Control of Manipulator assignment 2: Manipulator Geometry and Direct kinematic
clc;
clear;
close all;
addpath('include');


%% 1.
% You will need to define all the model matrices, and fill the so called iTj matrices inside BuildTree() function 
% Be careful to define the z-axis coinciding with the joint rotation axis,
% and such that the positive rotation is the same as showed in the CAD model you received.
geom_model = BuildTree();

% Useful initizializations
numberOfLinks = 7;                    % number of manipulator's links.
JointType =  zeros(1,numberOfLinks);                       % boolean that specifies two possible link types: Rotational, Prismatic.
bri= zeros(3,numberOfLinks);          % Basic vector of i-th link w.r.t. base
bTi0 = zeros(4,4,numberOfLinks);       % Trasformation matrix i-th link w.r.t. base
bTi1 = zeros(4,4,numberOfLinks);
bTi2 = zeros(4,4,numberOfLinks);
bTi3 = zeros(4,4,numberOfLinks);
iTj = zeros(4,4,1); %???
% Initial joint configuration 
q0 = [0,0,0,0,0,0,0];
q1 = [0,0,0,0,0,pi/2,0];
q2 = [0,pi/2,0,-pi/2,0,0,0];
q3=[pi/4, pi/2, -pi/8, -pi/2, pi/4, 2*pi/3, 0];

% Q1.1 and Q1.2
biTei0 = GetDirectGeometry(q0, geom_model, JointType);
biTei1 = GetDirectGeometry(q1, geom_model, JointType);
biTei2 = GetDirectGeometry(q2, geom_model, JointType);
biTei3 = GetDirectGeometry(q3, geom_model, JointType);

%Q1.3
for i =1:numberOfLinks
    bTi0(:,:,i)= GetTransformationWrtBase(biTei0, i);
    bTi1(:,:,i)= GetTransformationWrtBase(biTei1, i);
    bTi2(:,:,i)= GetTransformationWrtBase(biTei2, i);
    bTi3(:,:,i)= GetTransformationWrtBase(biTei3, i);
end
linkNumber=[1,2,3,4,5,6,7];
iTj = zeros(4,4,7,7);
for i=1:length(linkNumber)
    for j= 1:length(linkNumber)
        iTj(:,:,i,j) = GetFrameWrtFrame(i, j,biTei0);
    end
end
% 
for i = 1:numberOfLinks
    bri(:,i) = GetBasicVectorWrtBase(iTj, i);
end

% Q1.4
% Hint: use plot3() and line() matlab functions. 
% qi = q;
% qf = [];
% numberOfSteps =100;
% 
% for i = 1:numberOfSteps
% %-------------------MOVE----------------------%
% 
% end