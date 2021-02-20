% Here the simulator and Matlab communicate with each other through a ros
% master implemented in matlab

%% Variables declaration
torqueJoints = zeros(1,7);
forceEE = zeros(1,3);
torqueEE = zeros(1,3);

%% Start ROS master and nodes
rosinit
% Create ROS publishers and suscribers
[forcePub,force_msg] = rospublisher('/force');
% the pubisher is necessary so the suscriber can start in case the
% simulation hasn't started yet
listener = rossubscriber('/force');

%% Get data from simulator

for k=1:100
receive(listener,10); 
listener_msg = listener.LatestMessage.Data;
%fprintf('I heard: %s\n', listener_msg);  
%pause(1)

% Retrieve data here
msgValues = str2num(listener.LatestMessage.Data);
torqueJoints = msgValues(1:7);
forceEE = msgValues(8:10);
torqueEE = msgValues(11:13);

% Print out what it has received
fprintf('tJoints = %i %i %i %i %i %i %i, \n ',torqueJoints.');
fprintf('fEE = %i %i %i, ',forceEE.');
fprintf('tEE = %i %i %i, \n\n',torqueEE.');

pause(0.2)
end