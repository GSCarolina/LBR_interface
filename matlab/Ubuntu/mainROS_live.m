% Here the simulator and Matlab communicate with each other through a ros
% master implemented in matlab
% the data plotting is live

%% Variables declaration
posJoints = zeros(1,7);
posEE = zeros(1,3);
orienEE = zeros(1,3);
torqueJoints = zeros(1,7);
forceEE = zeros(1,3);
torqueEE = zeros(1,3);
% Previous values
posJoints_old = zeros(1,7);
posEE_old = zeros(1,3);
orienEE_old = zeros(1,3);
torqueJoints_old = zeros(1,7);
forceEE_old = zeros(1,3);
torqueEE_old = zeros(1,3);
%% Start ROS master and nodes
%rosinit
% Create ROS publishers and suscribers
forceSub = rossubscriber('/force');
posSub = rossubscriber('/position');

%% Start the plotting figure
max_total_pos = 0; min_total_pos = 0;
max_total_t = 0; min_total_t = 0;
tSimStep = 0.05;   % Simulation default step time = 50ms
nrJoints = 7;

figure
subplot(2,1,1)
title('Joints position during simulation')
xlabel('time [S]')
ylabel('Position [rad]')
grid on
hold on

subplot(2,1,2)
title('Joints torque during simulation')
xlabel('time [S]')
ylabel('Torque [N]')
grid on
hold on

%% Get data from simulator
tSim = 0;
max_total_pos = 0; min_total_pos = 0;
max_total_t = 0; min_total_t = 0;
tRecord = 10; % we want to record 10 seconds
tRecord_iterations = round(tRecord/tSimStep);

for k=1:tRecord_iterations
receive(forceSub,10); 
forceSub_msg = forceSub.LatestMessage.Data;
posSub_msg = posSub.LatestMessage.Data;

% Retrieve data here
msgValues = str2num(posSub_msg);
posJoints = msgValues(1:7)';
posEE = msgValues(8:10)';
orienEE = msgValues(11:13)';

msgValues = str2num(forceSub_msg);
torqueJoints = msgValues(1:7)';
forceEE = msgValues(8:10)';
torqueEE = msgValues(11:13)';


% Plotting
    local_value = max(posJoints);
    if (local_value > max_total_pos)
        max_total = local_value;
    end
    local_value = min(posJoints);
    if (local_value < min_total_pos)
        min_total_pos = local_value;
    end
    
    local_value = max(torqueJoints);
    if (local_value > max_total_t)
        max_total = local_value;
    end
    local_value = min(torqueJoints);
    if (local_value < min_total_t)
        min_total_t = local_value;
    end
%  Original    
% subplot(2,1,1)
% for i=1:nrJoints
%     plot(tSim, posJoints(i), '.');
% end
% xlim([0 tSim + tSimStep])
% ylim([min_total_pos max_total_pos]);
% 
% subplot(2,1,2)
% for i=1:nrJoints
%     plot(tSim, torqueJoints(i), '.');
% end
% xlim([0 tSim + tSimStep])
% ylim([min_total_t max_total_t]);
% 
% %update simulation values 
% tSim = tSim + tSimStep;  

subplot(2,1,1)
for i=1:nrJoints
    plot([tSim tSim+tSimStep], [posJoints_old(i) posJoints(i)]);
end
xlim([0, tSim+tSimStep])
ylim([min_total_pos max_total_pos]);

subplot(2,1,2)
for i=1:nrJoints
    plot([tSim tSim+tSimStep], [torqueJoints_old(i) torqueJoints(i)]);
end
xlim([0, tSim+tSimStep])
ylim([min_total_t max_total_t]);

%update simulation values 
tSim = tSim + tSimStep;  
posJoints_old = posJoints;
torqueJoints_old = torqueJoints;
end

