%------------ VERSION 1: PLOTTING AFTERWARDS -------------
%% Start connection with simulator
u = udp('172.31.1.147',30002); % only can be a client
fopen(u)
% state constants
ST_INIT = 0;
ST_RECORD = 1; 
ST_WAIT = 2;
ST_RUN = 3;
ST_STOP = 4;
%% Start up variables
stiffnessAx_acc = [];
damping_acc = [];
stiffnessAn_acc = [];

velJS_acc = [];
force_acc = [];

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
    status = dataDouble(1,1);
    
    if(length(dataDouble)>4 && (status == ST_RUN))
        status = dataDouble(1,1);
        stiffnessAx = dataDouble(2,1);
        stiffnessAn = dataDouble(3,1);
        damping = dataDouble(4,1);
        
        stiffnessAx_acc = [stiffnessAx_acc; stiffnessAx];
        stiffnessAn_acc = [stiffnessAn_acc; stiffnessAn];
        damping_acc = [damping_acc; damping];
        
        velJS = dataDouble(5,1);
        force = dataDouble(6,1);
        
        velJS_acc = [velJS_acc; velJS];
        force_acc = [force_acc; force];
    end
    
    % loop settings
    pause(tStep);
end

%fwrite(u,'ENDSOCKET')

%% Plot data
dataX = tStep*[0:(length(stiffnessAx_acc)-1)];
figure
% apf
subplot(2,2,1)
grid on
hold on
plot(dataX,stiffnessAx_acc','-b',dataX,stiffnessAn_acc','-r');
title('CS gains overtime')
xlabel('Time [s]')
xlim([dataX(1) dataX(end)])
ylabel('Stiffness [coeff]')
ylim([0 1.1*max(stiffnessAx_acc)])


subplot(2,2,2)
grid on
hold on
plot(dataX,damping_acc','-b');
title('CS gains overtime')
xlabel('Time [s]')
xlim([dataX(1) dataX(end)])
ylabel('Damping [coeff]')
ylim([0 1.1*max(damping_acc)])

% trajectory settings
subplot(2,2,3)
grid on
hold on
plot(dataX,velJS_acc','-b');
title('Trajectory velocity overtime')
xlabel('Time [s]')
xlim([dataX(1) dataX(end)])
ylabel('velocity [%override]')
ylim([0 1.1*max(velJS_acc)])


subplot(2,2,3)
grid on
hold on
plot(dataX,force_acc','-b');
title('Force at the ee overtime')
xlabel('Time [s]')
xlim([dataX(1) dataX(end)])
ylabel('Force [Nm]')
ylim([0 1.1*max(force_acc)])

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
subplot(2,2,1)
grid on
hold on
title('CS gains overtime')
xlabel('Time [s]')
ylabel('Stiffness [coeff]')

subplot(2,2,2)
grid on
hold on
plot(dataX,damping_acc','-b');
title('CS gains overtime')
xlabel('Time [s]')
ylabel('Damping [coeff]')

% trajectory settings
subplot(2,2,3)
grid on
hold on
title('Trajectory velocity overtime')
xlabel('Time [s]')
xlim([dataX(1) dataX(end)])
ylabel('velocity [%override]')

subplot(2,2,3)
grid on
hold on
title('Force at the ee overtime')
xlabel('Time [s]')
ylabel('Force [Nm]')

%% Start up variables
stiAx_old = 0; stiAn_old = 0; damp_old = 0;
velJS_old = 0; force_old = 0;

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
    status = dataDouble(1,1);
    
    if(length(dataDouble)>4 && (status == ST_RUN))
        stiffnessAx = dataDouble(2,1);
        stiffnessAn = dataDouble(3,1);
        damping = dataDouble(4,1);
        velJS = dataDouble(5,1);
        force = dataDouble(6,1);
        
        dataX =[k*tStep (k+1)*tStep];
        dataY = [stiAx_old stiAx; stiAn_old stiAn; 
                   damp_old damp; 
                   velJS_old velJS;
                   force_old force ];        
        % plot
        subplot(2,2,1)
        plot(dataX,dataY(1,:),'-b',dataX,dataY(2,:),'-r');
        subplot(2,2,3)
        plot(dataX, dataY(3,:),'-k');
        subplot(2,2,2)
        plot(dataX, dataY(4,:),'-k');
        subplot(2,2,4)
        plot(dataX, dataY(5,:),'-k');
        % update
        stiAx_old = stiAx; stiAn_old = stiAn; damp_old = damp;
        velJS_old = velJS; force_old = force;
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