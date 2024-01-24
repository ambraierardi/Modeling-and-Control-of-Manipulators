%% Exercises Modelling Part 1
% Rotation matrices, Equivalent angle-axis representations, Quaternions
addpath('include') %%DO NOT CHANGE STUFF INSIDE THIS PATH

%% Exercise 1
% Write a function called ComputeAngleAxis() implementing the Rodrigues Formula, 
% taking in input the geometric unit vector v and the rotation angle theta
% and returning the orientation matrix.
% and test it for the following cases:

% 1.1.


% 1.2.
theta=pi/4;
v=[1;0;0];
aRb = ComputeAngleAxis(theta, v);

disp('aRb ex 1.2:');disp(aRb);
plotRotation(theta,v,aRb);
disp('theta ex 1.2:');disp(theta);
disp('v ex 1.2:');disp(v);
% 1.3.
theta=pi/6;
v=[0;1;0];
aRb = ComputeAngleAxis(theta, v);

disp('aRb ex 1.3:');disp(aRb);
plotRotation(theta,v,aRb);
disp('theta ex 1.3:');disp(theta);
disp('v ex 1.3:');disp(v);
% 1.4.
theta=3*pi/4;
v=[0;0;1];
aRb = ComputeAngleAxis(theta, v);

disp('aRb ex 1.4:');disp(aRb);
plotRotation(theta,v,aRb);
disp('theta ex 1.4:');disp(theta);
disp('v ex 1.4:');disp(v);
% 1.5.
theta=2.8;
v=[0.3202; 0.5337; 0.7827];
aRb = ComputeAngleAxis(theta, v);

disp('aRb ex 1.5:');disp(aRb);
plotRotation(theta,v,aRb);
disp('theta ex 1.5:');disp(theta);
disp('v ex 1.5:');disp(v);
% 1.6.
theta=2*pi/3;
v=[0;1;0];
aRb = ComputeAngleAxis(theta, v);

disp('aRb ex 1.6:');disp(aRb);
plotRotation(theta,v,aRb);
disp('theta ex 1.6:');disp(theta);
disp('v ex 1.6:');disp(v);
% 1.7.
v=[0.25;-1.3;0.15];
theta=norm(v);
v=v/theta;
aRb = ComputeAngleAxis(theta, v);
disp('aRb ex 1.7:');disp(aRb);
plotRotation(theta,v,aRb);
disp('theta ex 1.7:');disp(theta);
disp('v ex 1.7:');disp(v);
% 1.8
v=[-1/4;-1/3;1/6];
theta=norm(v);
v=v/theta;
aRb = ComputeAngleAxis(theta, v);

disp('aRb ex 1.8:');disp(aRb);
plotRotation(theta,v,aRb);
disp('theta ex 1.8:');disp(theta);
disp('v ex 1.8:');disp(v);
%% Exercise 2
% 2.1. Write the relative rotation matrix aRb
% 2.2. Solve the Inverse Equivalent Angle-Axis Problem for the orientation matrix aRb. 
% 2.3. Repeat the exercises using the wRc instead of wRa (more general example)
% NB: check the notation used !

    % 2.1 
    % Initialize the rotation matrices, using the suggested notation
        %rotation matrix from <w> to frame <a>
        wRa=eye(3); %since <w> is aligned as <a>
        %rotation matrix from <w> to <b> (represent 90° around z)
        wRb=[0 -1 0; 1 0 0; 0 0 1];
    % Compute the rotation matrix between frame <a> and <b>
    aRb=transpose(wRa)*wRb;
    % 2.2
    % Compute the inverse equivalent angle-axis repr. of aRb 
    [theta, v] = ComputeInverseAngleAxis(aRb);
    % Plot Results
    disp('aRb ex 2.1:');disp(aRb);
    plotRotation(theta,v,aRb);
    disp('theta ex 2.2:');disp(theta);
    disp('v ex 2.2:');disp(v); 

    % 2.3

    % Compute the rotation matrix between frame <c> and <b>
    wTc=[0.835959 -0.283542 -0.46986 0; 0.271321 0.957764 -0.0952472 -1.23; 0.47703 -0.0478627 0.877583 14; 0 0 0 1];
    wRc=wTc(1:3,1:3);
    cRb=transpose(wRc)*wRb;
    % Compute inverse equivalent angle-axis repr. of cRb
    [theta, v] = ComputeInverseAngleAxis(cRb);
    % Plot Results
    plotRotation(theta,v,cRb);
    disp('theta ex 2.3:');disp(theta);
    disp('v ex 2.3:');disp(v); 

%% Exercise 3
% 3.1 Given two generic frames < w > and < b >, define the elementary 
% orientation matrices for frame < b > with respect to frame < w >, knowing
% that:
    % a. < b > is rotated of 45◦ around the z-axis of < w >
    % b. < b > is rotated of 60◦ around the y-axis of < w >
    % c. < b > is rotated of -30◦ around the x-axis of < w >
% 3.2 Compute the equivalent angle-axis representation for each elementary rotation
% 3.3 Compute the z-y-x (yaw,pitch,roll) representation using the already
% computed matrices and solve the Inverse Equivalent Angle-Axis Problem
% 3.4 Compute the z-x-z representation using the already computed matrices 
% and solve the Inverse Equivalent Angle-Axis Problem 

% 3.1
% hint: define angle of rotation in the initialization

    % a
        %rotation matrix from <w> to frame <b> by rotating around z-axes
        % wRb_z = ... ;
        psi=pi/4;
        wRb_z=[cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
        
    % b
        %rotation matrix from <w> to frame <b> by rotating around y-axes
        % wRb_y = ... ;
        theta=pi/3;
        wRb_y=[cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
        
    % c
        %rotation matrix from <w> to frame <b> by rotating around x-axes
        % wRb_x = ... ;
        phi=-pi/6;
        wRb_x=[1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
         disp('es 3.1:');disp(wRb_z);disp(wRb_y);disp(wRb_x);
    
% 3.2
    % a
        [theta, v] = ComputeInverseAngleAxis(wRb_z);
        % Plot Results
        plotRotation(theta,v,wRb_z);
        disp('theta ex 3.2.a:');disp(theta);
        disp('v ex 3.2.a:');disp(v); 
    % b
        [theta, v] = ComputeInverseAngleAxis(wRb_y);
        % Plot Results
        plotRotation(theta,v,wRb_y);
        disp('theta ex 3.2.b:');disp(theta);
        disp('v ex 3.2.b:');disp(v); 
    % c
        [theta, v] = ComputeInverseAngleAxis(wRb_x);
        % Plot Results
        plotRotation(theta,v,wRb_x);
        disp('theta ex 3.2.c:');disp(theta);
        disp('v ex 3.2.c:');disp(v); 
% 3.3 
    % Compute the rotation matrix corresponding to the z-y-x representation;
    Rxyz=wRb_z*wRb_y*wRb_x;
    % Compute equivalent angle-axis repr.
    [theta, v] = ComputeInverseAngleAxis(Rxyz);
    % Plot Results
    plotRotation(theta,v,Rxyz);
    disp('theta ex 3.3:');disp(theta);
    disp('v ex 3.3:');disp(v);
   
    
% 3.4
    % Compute the rotation matrix corresponding to the z-x-z representation;
    theta=pi/4;
    v=[0;0;1];
    Rz1 = ComputeAngleAxis(theta, v);
    theta=-pi/6;
    v=[1;0;0];
    Rx = ComputeAngleAxis(theta, v);
    theta=pi/4;
    v=[0;0;1];
    Rz2 = ComputeAngleAxis(theta, v);
    Rzxz=Rz1*Rx*Rz2;
	% Compute equivalent angle-axis repr.
    [theta, v] = ComputeInverseAngleAxis(Rzxz);
    % Plot Results
    plotRotation(theta,v,Rzxz);
    disp('theta ex 3.4:');disp(theta);
    disp('v ex 3.4:');disp(v);
    
   
%% Exercise 4
% 4.1 Represent the following quaternion with the equivalent angle-axis
% representation. q = 0.1647 + 0.31583i + 0.52639j + 0.77204k
% 4.2 Solve the Inverse Equivalent Angle-Axis Problem for the obtained orientation matrix
% 4.3 Repeat the exercise using the built-in matlab functions see:
% https://it.mathworks.com/help/nav/referencelist.html?type=function&category=coordinate-system-transformations&s_tid=CRUX_topnav
% CHECK IF THE RESULT IS THE SAME 
    %%%%%%%%%%%%

    %%%%%%%%%%%%%
    q0=0.1647;
    q1=0.31583;
    q2=0.52639;
    q3=0.77204;
    q=[q0;q1;q2;q3];
    % Compute the rotation matrix associated with the given quaternion
    rotMatrix = quatToRot(q0,q1,q2,q3);
    disp('rot matrix es 4.1');disp(rotMatrix)

    % solve using matlab functions quaternion(), rotmat(),
    % rotMatrix = ...
    quat=quaternion(q0,q1,q2,q3);
    rotMatrix=rotmat(quat,"point");


    % Evaluate angle-axis representation and display rotations - check if the
    % same results as before
    [theta, v] = ComputeInverseAngleAxis(rotMatrix);
    % Plot Results
    plotRotation(theta,v,rotMatrix);
    disp('rot matrix es 4.3');disp(rotMatrix)