%% Modelling and Control of Manipulator assignment 3 - Exercise 1: Jacobian matrix

% The same model of assignment 2
clear
close all
clc

addpath("include\")
geom_model = BuildTree();
numberOfLinks = size(geom_model,3); % number of manipulator's links.
linkType = zeros(numberOfLinks,1); % specify two possible link type: Rotational, Prismatic.
bTi = zeros(4,4,numberOfLinks);% Trasformation matrix i-th link w.r.t. base

% Initial joint configuration 
q = [1.3,1.3,1.3,1.3,1.3,1.3,1.3];
%% Compute direct geometry
%other initial configurations
q1 = [1.8, 1.8, 1.8, 1.8, 1.8, 1.8, 1.8];
q2 = [0.3, 1.4, 0.1, 2.0, 0, 1.3, 0];
q3 = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0];
q4 = [1, 1, 1, 1, 1, 1, 1];
biTei = GetDirectGeometry(q, geom_model, linkType);
biTei1 = GetDirectGeometry(q1, geom_model, linkType);
biTei2 = GetDirectGeometry(q2, geom_model, linkType);
biTei3 = GetDirectGeometry(q3, geom_model, linkType);
biTei4 = GetDirectGeometry(q4, geom_model, linkType);
% Computing end effector jacobian 
J=GetJacobian(biTei,linkType);
J1=GetJacobian(biTei1,linkType);
J2=GetJacobian(biTei2,linkType);
J3=GetJacobian(biTei3,linkType);
J4=GetJacobian(biTei4,linkType);
