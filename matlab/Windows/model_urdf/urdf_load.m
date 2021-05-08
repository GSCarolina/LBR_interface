%------------ MAIN CODE FOR MODELLING -------------
%clear
%clc
%Change this command to your repository address. 
folder_model = ['C:\Users\Carolina\Desktop\net_shared\LBR_interface\matlab\Windows\model_urdf'];
folder_functions = ['C:\Users\Carolina\Desktop\net_shared\LBR_interface\matlab\Windows\'];
addpath(folder_model);
addpath(folder_functions);
%% Load the Robot

%If the .urdf file was changed, uncomment the following lines:
%[lbr_med, solver] = initialization_UR3e('lbrMed.urdf')
%[lbr_med, solver] = initialization_robot('lbrMed.urdf')
%save([folder_lbr '\' 'lbrMed_param.mat'], 'lbr_med', 'solver');
%Otherwise
disp('Loading robot model...');
load([folder_model '\' 'lbrMed_param.mat']);
load([folder_model '\' 'eeConfig.mat']);

%% Generate a Set of Cartesian-Space Waypoints
% Structure of WayPoints matrix: 
%[x0 y0 z0;
% x1 y1 z0; ...
% xn yn zn];
disp('Generating trajectory...');
Waypoints = [ 0.50  0.05  0.07; 
              0.40  0.15  0.10; 
              0.35  0.30  0.10 ]; 
%              0.30  0.40  0.10 ; 
%              0.20  0.45  0.10; 
%              0.20  0.45  0.10];
n = 10;
Path = GeneratePath(n,Waypoints,'with');
size_Path = size(Path);

%% Convert Cartesian-Space Waypoints to Joint-Space Using Inverse Kinematics
% Define the orientation so that the end effector is oriented down
weights = [1 1 1 1 1 1];
eeName = 'LBRMed_wrist3';
Joint_init = zeros(7,1);
effector = eeDown;
JointPath = GenerateJointPath(effector,Path,Joint_init,solver,weights,eeName);

% Trajectory 2D and 3D Visualisation:
%PlotTrajectory(Waypoints, Path, JointPath);
%% Follow trajectory in Matlab
disp('Showing trajectory! :D');
% This is how the robot is supposed to move and follow the trajectory
PlotFollow(lbr_med, Waypoints, Path, JointPath, 0);

%% Modify end effector configuration
%Choose an end-effector contiguration:
% eeDown=[1 0 0 pi];      % gripper pointing down  
% eeLeft=[1 0 0 pi/2];    % gripper pointing to the left (world -Y)
% eeRight=[-1 0 0 pi/2];  % gripper pointing to the right (world +Y)
% eeFront=[0 1 0 pi/2];   % gripper pointing towards (world +X)
% eeBack=[0 -1 0 pi/2];   % gripper pointing inward (world -X)
% eeUp=[0 0 1 pi/2];      % gripper pointing up
%save('eeConfig.mat','eeDown','eeLeft','eeRight','eeFront','eeBack','eeUp');

