u = udp('172.31.1.147',30007);
fopen(u)

%% Start up variables
Uee_acc = [];
Ekee_acc = [];
Caxis_acc = [];
Damp_acc = [];

Qee_acc = [];

t = 0;
tStep = 0.1;

% some initialization
Uee_acc = [Uee_acc; 0 0 0 0 0 0];
Ekee_acc = [Ekee_acc; 0];
Caxis_acc = [Caxis_acc; 0 0 0];
Damp_acc = [Damp_acc; 0];

%% Plot set up
figure('Name', 'Potential and kinetic energy on the run')
subplot(2,2,1)
grid on 
hold on
title('Potential energy')
xlabel('Time [s]')
ylabel('Uee [unit]')

subplot(2,2,3)
grid on 
hold on
title('Caxis gains')
xlabel('Time [s]')
ylabel('K [coeff]')

subplot(2,2,2)
grid on 
hold on
title('Kinetic energy')
xlabel('Time [s]')
ylabel('Ekee [unit]')

subplot(2,2,4)
grid on 
hold on
title('Damping gain')
xlabel('Time [s]')
ylabel('Damping [coeff]')

h = animatedline;
%% main loop
for k=0:100
    % exchange data with simulator
    message = [num2str(k) ';GAINS;'];
    fwrite(u,message)
    A = fread(u, 40);
    recMsg = ['Received: ' char(A')];
    %disp(recMsg);
    
    % process data
    dataDouble = (str2num(char(A')));    
    Uee = dataDouble(1:6,1)';
    Caxis = dataDouble(7:9,1)';
    Ekee = dataDouble(10,1)';
    Damp = dataDouble(11,1)';
    q17 = dataDouble(12:18,1)';
    
    Uee_acc = [Uee_acc; Uee];
    Ekee_acc = [Ekee_acc; Ekee];
    Caxis_acc = [Caxis_acc; Caxis];
    Damp_acc = [Damp_acc; Damp];
    
    % plot realtime
    dataX = [t t+tStep];
    
    subplot(2,2,1)
    plot(dataX,[Uee_acc(end-1,1) Uee_acc(end,1)],'-b',dataX,[Uee_acc(end-1,2) Uee_acc(end,2)],'-r',dataX,[Uee_acc(end-1,3) Uee_acc(end,3)],'-g');
    drawnow
    
    subplot(2,2,3)
    plot(dataX,[Caxis_acc(end-1,1) Caxis_acc(end,1)],'-b',dataX,[Caxis_acc(end-1,2) Caxis_acc(end,2)],'-r',dataX,[Caxis_acc(end-1,3) Caxis_acc(end,3)],'-g');
    drawnow

    subplot(2,2,2)
    plot(dataX,[Ekee_acc(end-1,1) Ekee_acc(end,1)],'-k');
    drawnow

    subplot(2,2,4)
    plot(dataX,[Damp_acc(end-1,1) Damp_acc(end,1)],'-k');
    drawnow
    
    % add legend
    if(k==0)
        subplot(2,2,1)
        legend('X','Y','Z','AutoUpdate','off')
        subplot(2,2,3)
        legend('X','Y','Z','AutoUpdate','off')
        subplot(2,2,2)
        legend('Ek','AutoUpdate','off')
        subplot(2,2,4)
        legend('D','AutoUpdate','off')
    end
    % loop settings
    t = t+tStep;
    pause(tStep);    
end

