% Here the simulator and Matlab communicate with each other through a ros
% master implemented in matlab

%% Variables declaration
posJoints = zeros(1,7);
posEE = zeros(1,3);
orienEE = zeros(1,3);

torqueJoints = zeros(1,7);
forceEE = zeros(1,3);
torqueEE = zeros(1,3);

%% Start ROS master and nodes
%rosinit
% Create ROS publishers and suscribers
[forcePub,force_msg] = rospublisher('/force');
[posPub,pos_msg] = rospublisher('/position');
% the pubisher is necessary so the suscriber can start in case the
% simulation hasn't started yet
forceSub = rossubscriber('/force');
posSub = rossubscriber('/position');

%% Get data from simulator
pJoints_record = [];
tJoints_record = [];
pEE_record = [];
ftEE_record = [];

for k=1:1000
receive(forceSub,10); 
forceSub_msg = forceSub.LatestMessage.Data;
posSub_msg = posSub.LatestMessage.Data;
%fprintf('I heard: %s\n', listener_msg);  
%pause(0.2)

% Retrieve data here
msgValues = str2num(posSub_msg);
posJoints = msgValues(1:7)';
posEE = msgValues(8:10)';
orienEE = msgValues(11:13)';

msgValues = str2num(forceSub_msg);
torqueJoints = msgValues(1:7)';
forceEE = msgValues(8:10)';
torqueEE = msgValues(11:13)';

% Acumulate/save values for plotting it later
pJoints_record = [pJoints_record; posJoints];
tJoints_record = [tJoints_record; torqueJoints];
pEE_record = [pEE_record; posEE, orienEE];
ftEE_record = [ftEE_record; forceEE, torqueEE];

% Print out what it has received
%fprintf('tJoints = %i %i %i %i %i %i %i, \n ',torqueJoints.');
%fprintf('fEE = %i %i %i, ',forceEE.');
%fprintf('tEE = %i %i %i, \n\n',torqueEE.');

end

%% Plot
tSimStep = 0.05;   % Simulation default step time = 50ms
[tSim_samples nrJoints] = size(pJoints_record);
tSim = [0:tSim_samples-1]*tSimStep;

% Plot joints: position and torques
figure
subplot(2,1,1)
title('Joints position during simulation')
xlabel('time [S]')
ylabel('Position [rad]')
grid on
hold on
max_total = 0; min_total = 0;
for i=1:nrJoints
    plot(tSim,abs(pJoints_record(:,i)'));
    
    local_max = max(pJoints_record(:,i));
    if (local_max > max_total)
        max_total = local_max;
    end
    
    local_min = min(pJoints_record(i,:));
    if (local_min < min_total)
        min_total = local_min;
    end
end
xlim([0 tSim(end)]);
ylim([min_total max_total]);

subplot(2,1,2)
title('Joints torque during simulation')
xlabel('time [S]')
ylabel('Torque [N]')
grid on
hold on
max_total = 0; min_total = 0;
for i=1:nrJoints
    plot(tSim,abs(tJoints_record(:,i)'));
    
    local_max = max(tJoints_record(:,i));
    if (local_max > max_total)
        max_total = local_max;
    end
    
    local_min = min(tJoints_record(:,i));
    if (local_min < min_total)
        min_total = local_min;
    end
end
xlim([0 tSim(end)]);
ylim([min_total max_total]);
