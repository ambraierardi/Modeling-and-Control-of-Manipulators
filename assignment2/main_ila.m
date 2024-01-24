%% Modelling and Control of Manipulator assignment 2: Manipulator Geometry and Direct kinematic
clc;
clear;
close all;
addpath('include');


%% Useful initizializations
geom_model = BuildTree();
numberOfLinks = 7;                    % number of manipulator's links.
linkType = zeros(1,numberOfLinks);                         % boolean that specifies two possible link types: Rotational, Prismatic.
bri= zeros(3,numberOfLinks);        % Basic vector of i-th link w.r.t. base
bTi0 = zeros(4,4,numberOfLinks);     % Trasformation matrix i-th link w.r.t. base
bTi1 = zeros(4,4,numberOfLinks);
bTi2 = zeros(4,4,numberOfLinks);
bTi3 = zeros(4,4,numberOfLinks);

iTj = zeros(4,4,1);
% Initial joint configuration 
q = [0,0,0,0,0,0,0];

% Q1.1 and Q1.2
q0 = [0,0,0,0,0,0,0];
biTei0 = GetDirectGeometry(q0,geom_model,linkType);

q1 = [0,0,0,0,0,pi/2,0];
biTei1 = GetDirectGeometry(q1,geom_model,linkType);

q2 = [0,pi/2,0,-pi/2,0,0,0];
biTei2 = GetDirectGeometry(q2,geom_model,linkType);

q3= [pi/4,pi/2,-pi/8,-pi/2,pi/4,2/3*pi,0];
biTei3 = GetDirectGeometry(q3,geom_model,linkType);

%Q1.3
for i =1:numberOfLinks
    bTi0(:,:,i)= GetTransformationWrtBase(biTei0,i);
    bTi1(:,:,i)= GetTransformationWrtBase(biTei1,i);
    bTi2(:,:,i)= GetTransformationWrtBase(biTei2,i);
    bTi3(:,:,i)= GetTransformationWrtBase(biTei3,i);
end


%%check it
cell_arr0 = cell(7,7);
cell_arr1 = cell(7,7);
cell_arr2 = cell(7,7);
cell_arr3 = cell(7,7);

for i = 1:numberOfLinks
    for j = 1:numberOfLinks
        iTj0(:,:) = GetFrameWrtFrame(i,j,bTi0);
        iTj1(:,:) = GetFrameWrtFrame(i,j,bTi1);
        iTj2(:,:) = GetFrameWrtFrame(i,j,bTi2);
        iTj3(:,:) = GetFrameWrtFrame(i,j,bTi3);
        cell_arr0{i,j} = iTj0;
        cell_arr1{i,j} = iTj1;
        cell_arr2{i,j} = iTj2;
        cell_arr3{i,j} = iTj3;
    end
end

for i = 1:numberOfLinks
    bri0(:,i) = GetBasicVectorWrtBase(biTei0,i);
    bri1(:,i) = GetBasicVectorWrtBase(biTei1,i);
    bri2(:,i) = GetBasicVectorWrtBase(biTei2,i);
    bri3(:,i) = GetBasicVectorWrtBase(biTei3,i);
end

% Q1.4
% Hint: use plot3() and line() matlab functions. 
numberOfSteps =100;
qi1 = zeros(1,numberOfLinks);
qf1 = [pi/4 pi/2,-pi/8,-pi/2,pi/4,2/3*pi,0];
qstep1=(qf1-qi1)/numberOfSteps;
qstep1_reference=qstep1;
qi2=[0,pi/2,0,-pi/2,0,0,0];
qf2= zeros(1,numberOfLinks);
qstep2=(qf2-qi2)/100;
qstep2_reference=qstep2;
qi3= [1.3, 0.1, 0.1, 1, 0.2, 0.3, 1.3];
qf3= [2, 2, 2, 2, 2, 2, 2];
qstep3=(qf3-qi3)/100;
qstep3_reference=qstep3;

axislimits=[-0.9, 0.9, -0.7, 0.7, -0.5, 1];
figure(1)
for i = 1:numberOfSteps
%-------------------MOVE----------------------%
biTei = GetDirectGeometry(qstep1+qi1,geom_model,linkType);
for j=1:numberOfLinks
    r(:,j) = GetBasicVectorWrtBase(biTei, j);
    plot3(r(1,:),r(2,:),r(3,:),'-o','Color','b', 'MarkerSize',5,'LineWidth',1.5)
    grid on
    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
    axis(axislimits);
    title("Q1.4");
    subtitle("Configuration: qi1=[0,0,0,0,0,0,0]")
end
qstep1=qstep1+qstep1_reference;
pause(0.001)
end
%%
figure(2)
for i = 1:numberOfSteps
%-------------------MOVE----------------------%
biTei = GetDirectGeometry(qstep2+qi2,geom_model,linkType);
for j=1:numberOfLinks
    r(:,j) = GetBasicVectorWrtBase(biTei, j);
    plot3(r(1,:),r(2,:),r(3,:),'-o','Color','b', 'MarkerSize',5,'LineWidth',1.5)
    grid on
    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
    axis(axislimits);
    title("Q1.4");
    subtitle("Configuration: qi2=[0,pi/2,0,-pi/2,0,0,0]")
end
qstep2=qstep2+qstep2_reference;
pause(0.001)
end
%%
figure(3)
for i = 1:numberOfSteps
%-------------------MOVE----------------------%
biTei = GetDirectGeometry(qstep3+qi3,geom_model,linkType);
for j=1:numberOfLinks
    r(:,j) = GetBasicVectorWrtBase(biTei, j);
    plot3(r(1,:),r(2,:),r(3,:),'-o','Color','b', 'MarkerSize',5,'LineWidth',1.5)
    grid on
    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
    axis(axislimits);
    title("Q1.4");
    subtitle("Configuration: qi3= [1.3, 0.1, 0.1, 1, 0.2, 0.3, 1.3]")
end
qstep3=qstep3+qstep3_reference;
pause(0.001)
end
%%
%1.5
%first configuration
axislimits=[-0.9, 0.9, -0.7, 0.7, -0.5, 1];
figure(4)
qstep=zeros(1,numberOfLinks);
for k=1:numberOfLinks
    qstep(k)=qstep1_reference(k);
    for i = 1:numberOfSteps
        %-------------------MOVE----------------------%
        biTei = GetDirectGeometry(qstep+qi1,geom_model,linkType);
        for j=1:numberOfLinks
            r(:,j) = GetBasicVectorWrtBase(biTei, j);
            plot3(r(1,:),r(2,:),r(3,:),'-o','Color','b', 'MarkerSize',5,'LineWidth',1.5)
            axis equal
            xlabel('x')
            ylabel('y')
            zlabel('z')
            grid on
            axis(axislimits);
            title("Q1.5");
            subtitle("Configuration: qi1=[0,0,0,0,0,0,0]")
        end
        qstep(k)=qstep(k)+qstep1_reference(k);
        pause(0.001)
    end
end
%%
%second configuration
axislimits=[-0.9, 0.9, -0.7, 0.7, -0.5, 1];
figure(5)
qstep=zeros(1,numberOfLinks);
for k=1:numberOfLinks
    qstep(k)=qstep2_reference(k);
    for i = 1:numberOfSteps
        %-------------------MOVE----------------------%
        biTei = GetDirectGeometry(qstep+qi2,geom_model,linkType);
        for j=1:numberOfLinks
            r(:,j) = GetBasicVectorWrtBase(biTei, j);
            plot3(r(1,:),r(2,:),r(3,:),'-o','Color','b', 'MarkerSize',5,'LineWidth',1.5)
            axis equal
            xlabel('x')
            ylabel('y')
            zlabel('z')
            grid on
            axis(axislimits);
            title("Q1.5");
            subtitle("Configuration: qi2=[0,pi/2,0,-pi/2,0,0,0]")
        end
        qstep(k)=qstep(k)+qstep2_reference(k);
        pause(0.001)
    end
end
%%
%third configuration
axislimits=[-0.9, 0.9, -0.7, 0.7, -0.5, 1];
figure(6)
qstep=zeros(1,numberOfLinks);
for k=1:numberOfLinks
    qstep(k)=qstep3_reference(k);
    for i = 1:numberOfSteps
        %-------------------MOVE----------------------%
        biTei = GetDirectGeometry(qstep+qi3,geom_model,linkType);
        for j=1:numberOfLinks
            r(:,j) = GetBasicVectorWrtBase(biTei, j);
            plot3(r(1,:),r(2,:),r(3,:),'-o','Color','b', 'MarkerSize',5,'LineWidth',1.5)
            axis equal
            xlabel('x')
            ylabel('y')
            zlabel('z')
            grid on
            axis(axislimits);
            title("Q1.5");
            subtitle("Configuration: qi3= [1.3, 0.1, 0.1, 1, 0.2, 0.3, 1.3]")
        end
        qstep(k)=qstep(k)+qstep3_reference(k);
        pause(0.001)
    end
end