%% Modelling and Control of Manipulator assignment 3 - Exercise 2 and 3: Inverse Kinematic Control
addpath('include')
model = load("panda.mat"); % don't worry about eventual warnings!
% Simulation Parameters
ts = 0.5;
t_start = 0.0;
t_end = 30.0;
t = t_start:ts:t_end;

% Initial Joints configuration
q_init = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';
% Joint limits
qmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
qmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];
% Function that gives the transformation from <base> to <e-e>, given a
% configuration of the manipulator
bTe = getTransform(model.franka,[q_init',0,0],'panda_link7');%DO NOT EDIT 
bRe=bTe(1:3,1:3);

% Tool frame definition
phi=deg2rad(-44.98);
eOt = [0;0;0.2104]; %meters
eRt = [cos(phi), -sin(phi), 0; sin(phi), cos(phi), 0; 0, 0, 1];
eTt = [eRt,eOt;zeros(1,3),1];
%bTt = getTransform(model.franka,[q_init',0,0],'panda_link7');
bTt=bTe*eTt;

% Goal definition 
bOg = [0.55; -0.3; 0.2]; %meters
theta=pi/6;
eRg=[cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];

% Switch between the two cases (with and without the tool frame)
tool = false; % change to true for using the tool
if tool == true  % if controlling the tool frame
    tRg=[cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
    bRt=bRe*eRt;
    bRg=bRt*tRg;
    bTg = [bRg, bOg;zeros(1,3), 1];
else
    bRg=bRe*eRg;
    bTg = [bRg, bOg;zeros(1,3), 1];
end   

% Control Proportional Gain 
angular_gain = 0.2;
linear_gain = 0.2;
% Preallocation variables
x_dot = zeros(6,1);
lin_err = zeros(3,1);
ang_err = zeros(3,1); 
% Start the inverse kinematic control  
q = q_init;

%% Simulation Loop
for i = t
    
    if tool == true %compute the error between the tool frame and goal frame
        
        % Computing transformation matrix from base to end effector 
        bTe = getTransform(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        tmp = geometricJacobian(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT

        eRt = [cos(phi), -sin(phi), 0; sin(phi), cos(phi), 0; 0, 0, 1];
        eTt = [eRt,eOt;zeros(1,3),1];
        bTt=bTe*eTt;

        bJe = tmp(1:6,1:7); %DO NOT EDIT
        A=[0, -eOt(3), eOt(2); eOt(3), 0, -eOt(1);-eOt(2),eOt(1),0];
        bJt = [eye(3),zeros(3);A,eye(3)]*bJe;
        tTg=pinv(bTt)*bTg;
        lin_err = bTt(1:3,1:3)*tTg(1:3,4);
        [theta_rot,v] = ComputeInverseAngleAxis(tTg(1:3,1:3));
        ang_err = bTt(1:3,1:3)*theta_rot*v;
        
    else % compute the error between the e-e frame and goal frame
        % Computing transformation matrix from base to end effector 
        bTe = getTransform(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        % Computing end effector jacobian w.r.t. base
        tmp = geometricJacobian(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        bJe = tmp(1:6,1:7); %DO NOT EDIT
        eTg=pinv(bTe)*bTg;
        lin_err = bTe(1:3,1:3)*eTg(1:3,4);
        [theta_rot,v] = ComputeInverseAngleAxis(eTg(1:3,1:3));
        ang_err = bTe(1:3,1:3)*theta_rot*v;
    end
    
    %% Compute the reference velocities
    %supposing the goal to be steady in its position
    bOmegae0_des=angular_gain*ang_err;
    bve0_des=linear_gain*lin_err;
    x_dot=[bOmegae0_des; bve0_des];
    %% Compute desired joint velocities 
    if tool==false
        q_dot=pinv(bJe)*x_dot;
    else
        q_dot=pinv(bJt)*x_dot;
    end
    %% Simulate the robot - implement the function KinematicSimulation()
    q = KinematicSimulation(q(1:7), q_dot,ts, qmin, qmax);
    
    % DO NOT EDIT - plot the robot moving
    %switch visuals to off for seeing only the frames
    show(model.franka,[q',0,0],'visuals','on');
    hold on
    if tool == true
        %set the window size of the figure to "full-screen" for a better visualization
        plot3(bTt(1,4),bTt(2,4),bTt(3,4),'go','LineWidth',15);
        plot3(bOg(1),bOg(2),bOg(3),'ro','LineWidth',5);
    else
        plot3(bTe(1,4),bTe(2,4),bTe(3,4),'go','LineWidth',15);
        plot3(bOg(1),bOg(2),bOg(3),'ro','LineWidth',5);
    end
    drawnow
    if(norm(x_dot) < 0.001)
        disp('REACHED THE REQUESTED GOAL POSITION')
        break
    end
    hold off
end
