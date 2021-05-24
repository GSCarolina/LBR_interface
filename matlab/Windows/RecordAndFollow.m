%------------ VERSION 1: PLOTTING AFTERWARDS -------------
%% Start connection with simulator
u = udp('172.31.1.147',30002); % only can be a client
fopen(u)

%% Start up variables
stiffnessAx_acc = [];
damping_acc = [];
stiffnessAn_acc = [];

%% main loop
tStep = 0.01;
for k=0:1000
    % exchange data with simulator
    message = ['1;MATLAB;'];
    fwrite(u,message)
    A = fread(u, 128);
    recMsg = ['Received: ' char(A')];
    %disp(recMsg);
    
    % process data
    dataDouble = (str2num(char(A')));    
    
    if(length(dataDouble)>2)
        stiffnessAx = dataDouble(1,1);
        stiffnessAn = dataDouble(2,1);
        damping = dataDouble(3,1);
        
        stiffnessAx_acc = [stiffnessAx_acc; stiffnessAx];
        stiffnessAn_acc = [stiffnessAn_acc; stiffnessAn];
        damping_acc = [damping_acc; damping];
        
    end
    
    % loop settings
    pause(tStep);
end

%fwrite(u,'ENDSOCKET')

%% Plot data
dataX = tStep*[0:(length(stiffnessAx_acc)-1)];
figure
% apf
subplot(1,2,1)
grid on
hold on
plot(dataX,stiffnessAx_acc','-b',dataX,stiffnessAn_acc','-r');
title('CS gains overtime')
xlabel('Time [s]')
xlim([dataX(1) dataX(end)])
ylabel('Stiffness [coeff]')
ylim([0 1.1*max(stiffnessAx_acc)])


subplot(1,2,2)
grid on
hold on
plot(dataX,damping_acc','-b');
title('CS gains overtime')
xlabel('Time [s]')
xlim([dataX(1) dataX(end)])
ylabel('Damping [coeff]')
ylim([0 1.1*max(damping_acc)])

%% End connection with simulator
fclose(u)
delete(u)

%%
%------------ VERSION 2: PLOTTING LIVE -------------
%% Start connection with simulator
u = udp('172.31.1.147',30002); % only can be a client
fopen(u)

%% start plots
figure
% apf
subplot(1,2,1)
grid on
hold on
title('CS gains overtime')
xlabel('Time [s]')
ylabel('Stiffness [coeff]')

subplot(1,2,2)
grid on
hold on
title('CS gains overtime')
xlabel('Time [s]')
ylabel('Damping [coeff]')

%% Start up variables
stiAx_old = 0;
stiAn_old = 0;
damp_old = 0;

%% main loop
tStep = 1;
for k=0:10
    % exchange data with simulator
    message = ['1;MATLAB;'];
    fwrite(u,message)
    A = fread(u, 128);
    recMsg = ['Received: ' char(A')];
    %disp(recMsg);
    
    % process data
    dataDouble = (str2num(char(A')));    
    
    if(length(dataDouble)>2)
        stiAx = dataDouble(1,1);
        stiAn = dataDouble(2,1);
        damp = dataDouble(3,1);
        
        dataX =[k*tStep (k+1)*tStep];
        dataY = [stiAx_old stiAx; stiAn_old stiAn; damp_old damp];        
        % plot
        subplot(1,2,1)
        plot(dataX,dataY(1,:),'-b',dataX,dataY(2,:),'-r');
        subplot(1,2,2)
        plot(dataX, dataY(3,:),'-k');
        % update
        stiAx_old = stiAx; stiAn_old = stiAn; damp_old = damp;
    end
    
    % loop settings
    pause(tStep);
end

%fwrite(u,'ENDSOCKET')

%% Plot data
dataX = tStep*[0:(length(stiffnessAx_acc)-1)];
figure
% apf
subplot(1,2,1)
grid on
hold on
plot(dataX,stiffnessAx_acc','-b',dataX,stiffnessAn_acc','-r');
title('CS gains overtime')
xlabel('Time [s]')
xlim([dataX(1) dataX(end)])
ylabel('Stiffness [coeff]')
ylim([0 1.1*max(stiffnessAx_acc)])


subplot(1,2,2)
grid on
hold on
plot(dataX,damping_acc','-b');
title('CS gains overtime')
xlabel('Time [s]')
xlim([dataX(1) dataX(end)])
ylabel('Damping [coeff]')
ylim([0 1.1*max(damping_acc)])

%% End connection with simulator
fclose(u)
delete(u)