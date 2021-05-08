%------------ MAIN CODE FOR TRAJECTORY -------------
%% Adding dependencies
%clear
%clc
%Change this command to your repository address. 
folder_model = ['C:\Users\Carolina\Desktop\net_shared\LBR_interface\matlab\Windows\model_urdf'];
folder_energy = ['C:\Users\Carolina\Desktop\net_shared\LBR_interface\matlab\Windows\energy'];
folder_trajectory = ['C:\Users\Carolina\Desktop\net_shared\LBR_interface\matlab\Windows\trajectory'];
addpath(folder_model);
addpath(folder_energy);
addpath(folder_trajectory);

load('LBRMed_param.mat');
%% Load recordings
% Recordings
folder_record = ['C:\Users\Carolina\Desktop\net_shared\LBR_interface\recording'];
addpath(folder_record);
data_dirJS = dir([folder_record '\JS_*.txt']);
data_dirCS = dir([folder_record '\CS_*.txt']);
[amount dummy] = size(data_dirJS);     
[amountcs dummy] = size(data_dirCS);     
% amount = how many recordings are on the folder

%% Convert from JS to CS
full_name = [data_dirJS(1).folder '\' data_dirJS(1).name]; %Just take the last one
JS_data = (load(full_name))';
JS_points = round(length(JS_data)/7);

% X axis are switched in this environment
joints = zeros(7,JS_points);
for j=1:7
    joints(j,:) = -JS_data(j:7:end);
end

% cartesian space
full_name = [data_dirCS(1).folder '\' data_dirCS(1).name]; %Just take the last one
CS_data = (load(full_name))';
CS_points = round(length(CS_data)/6);
CS_path = zeros(3,JS_points);
for j=1:3
    CS_path(j,:) = 0.001 * CS_data(j:6:end); % from mm to m
    CS_path(j,3) = CS_path(j,3) + 0.922; % shift Z axis
end
PlotFollow(robert, CS_path', CS_path', joints',0,'robert')


Path=[]; Waypoints = [];
for j=1:JS_points
    TCP_transform = getTransform(robert,joints(:,j),'LBRMed_base','LBRMed_wrist3');
    point = TCP_transform(1:3,4)';
    
    %Path = [Path; point(1) -point(2) point(3)];
    %Waypoints = [Waypoints; point(1) -point(2) point(3)];
    
    Path = [Path; point(1) point(2) point(3)];
    Waypoints =[Waypoints; point(1) point(2) point(3)];
end

PlotFollow(robert, Waypoints, Path, joints',10,'robert')
