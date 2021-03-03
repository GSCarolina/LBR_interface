% Listen on all available Ethernet interfaces at local port 8000.
% Specify a LocalHost (host name or IP address) if known
u = udp('172.31.1.147',30007);
fopen(u)
%% Init
% Plot var init
tSim = 0;
max_total_pos = 0; min_total_pos = 0;
max_total_t = 0; min_total_t = 0;
tSimStep = 0.05;   % Simulation default step time = 50ms
nrJoints = 7;
posJoints = zeros(1,nrJoints);
posJoints_old = zeros(1,nrJoints);

% Start plot (one single big=
figure
title(['Joints ']);
ylim([-150 150]);
xlabel('time [S]')
ylabel('Position [deg]')
grid on
hold on
% multiple
for k=1:7
    subplot(7,1,k)
    title(['Joint ' num2str(k)]);
    xlabel('time [S]')
    ylabel('Position [deg]')
    grid on
    hold on
    
    % degreees
    %if((k==1)||(k==3)||(k==5))
    %    ylim([-170 170]);
    %else
    %    ylim([-120 120]);
    %end
    %ylim([-150 150]);
end
%% ROS for the simulator
% Static IP address of the simulator
%rosinit('172.31.1.210');
[posPub,posMsg] = rospublisher('/realQ');
[torPub,torMsg] = rospublisher('/realT');


%% main loop
for k=0:50000
    message = [num2str(k) ';STATUS;'];
    fwrite(u,message)
    A = fread(u, 40);
    %recMsg = ['Received: ' char(A')];
    %disp(recMsg);
    %dataChar = char(A');
    dataDouble = (str2num(char(A')));
    posJointsRad = dataDouble(1:7,1)';
    torJoints = dataDouble(8:13,1)';
    
    posJointsChar = char(A');
    [posPub,posMsg] = rospublisher('/position');
    posJointsRad = (str2num(posJointsChar))';   
    posJoints = posJointsRad*57.2958; %transform to degrees
    % Plotting
%    local_value = max(posJoints);
%    if (local_value > max_total_pos)
%        max_total = local_value;
%    end
%    local_value = min(posJoints);
%    if (local_value < min_total_pos)
%        min_total_pos = local_value;
%    end
    
    %for i=1:nrJoints
    %subplot(7,1,i)
    %plot([tSim tSim+tSimStep], [posJoints_old(i) posJoints(i)],'b');
    %plot([tSim tSim+tSimStep], [posJoints_old(i) posJoints(i)]);
    %end
    %title({['A1 ' num2str(posJoints(1))] ,['A2 ' num2str(posJoints(2))],['A3 ' num2str(posJoints(3))],['A4 ' num2str(posJoints(4))], ['A5 ' num2str(posJoints(5))],['A6 ' num2str(posJoints(6))],['A7 ' num2str(posJoints(7))]});
    %xlim([0, tSim+tSimStep])

    % send joints position to simulator
    posMsg.Data = char(posJointsRad);
    send(posPub,posMsg); 
            
    %update simulation values 
    tSim = tSim + tSimStep;  
    posJoints_old = posJoints;
    
    pause(0.1);
end

fclose(u)
delete(u)
