%------------ CONNECTION TO COPPELIASIM -------------
clear
clc
% Add CoppeliaSim dependencies for Matlab
copp_folder_headers = 'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\remoteApiBindings\matlab\matlab';
copp_folder_lib = 'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\remoteApiBindings\lib\lib\Windows';
addpath(copp_folder_headers);
addpath(copp_folder_lib);

%% Target joint-space positions
targetPos1=[90*pi/180,90*pi/180,170*pi/180,-90*pi/180,90*pi/180,90*pi/180,0];
targetPos2=[-90*pi/180,90*pi/180,180*pi/180,-90*pi/180,90*pi/180,90*pi/180,0];
targetPos3=zeros(1,7);

targetPos_all = [targetPos1; targetPos2; targetPos3];
[nrPoints nrJoints] = size(targetPos_all);
%% Start client
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19000,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');
    
    % Get joints handlers
    h = zeros(1,7);
    for k=1:7
    [r,h(k)]=sim.simxGetObjectHandle(clientID,['LBRMed_joint' num2str(k)],sim.simx_opmode_blocking);
    end
    
    % Move the robot to different target positions
    for j=1:nrPoints
        jointPos = targetPos_all(j,:);  % Select a target position
        
        % Move robot
        disp(['Moving to target position nr.' num2str(j)]);
        for k=1:7
            sim.simxSetJointTargetPosition(clientID,h(k),jointPos(k),sim.simx_opmode_streaming);
        end
        
        pause(3);
    end
else
    disp('Failed connecting to remote API server');
end

% Finish
sim.delete(); % call the destructor!
disp('Program ended');