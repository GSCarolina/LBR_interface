%------------ MAIN CODE FOR SIMULATOR -------------
%% Start connection with simulator
u = udp('127.0.0.1',30002); % only can be a client
fopen(u)

%% Start up variables
uIn_acc = [];
uOut_acc = [];
xIn_acc = [];
xOut_acc = [];

% main loop
tStep = 0.001;
for k=0:10000
    % exchange data with simulator
    message = ['Matlab socket'];
    fwrite(u,message)
    A = fread(u, 128);
    recMsg = ['Received: ' char(A')];
    %disp(recMsg);
    
    % process data
    dataDouble = (str2num(char(A')));    
   
    if(length(dataDouble)>11)
        uOut = dataDouble(1:3,1)';
        uIn = dataDouble(4:6,1)';
        xIn = dataDouble(7:9,1)';
        xOut = dataDouble(10:12,1)';
        
        uIn_acc = [uIn_acc; uIn];
        uOut_acc = [uOut_acc; uOut];
        xIn_acc = [xIn_acc; xIn];
        xOut_acc = [xOut_acc; xOut];
    end
    
    % loop settings
    pause(tStep);
end

fwrite(u,'ENDSOCKET')

%% Plot data
dataX = tStep*[0:(length(uIn_acc(:,3))-1)];
figure

% end-effector position
% subplot(1,2,1)
% grid on
% hold on
% plot3(xIn_acc(:,1)',xIn_acc(:,2)',xIn_acc(:,3)','-r','LineWidth', 2);
% plot3(xOut_acc(:,1)',xOut_acc(:,2)',xOut_acc(:,3)','-k','LineWidth', 2);
% 
% title('Path potential energy')
% legend('Given path','Corrected path')
% xlabel('Position [m]')
% ylabel('Position [m]')
% zlabel('Position [m]')
% %xlim([0.9*min(xIn_acc(:,1)) 1.1*max(xIn_acc(:,1))])
% %ylim([0.9*min(xIn_acc(:,2)) 1.1*max(xIn_acc(:,2))])
% %zlim([0.9*min(xIn_acc(:,3)) 1.1*max(xIn_acc(:,3))])
% hold off

% energy
% subplot(1,2,2)
grid on
hold on
plot(dataX,uIn_acc(:,3)','-r');
plot(dataX,uOut_acc(:,3)','-k');

title('Path potential energy')
legend('Given path','Corrected path')
xlabel('Time [s]')
xlim([dataX(1) dataX(end)])
ylabel('u [unit]')
ylim([0 1.1*max(uIn_acc(:,3))])
hold off

%% End connection with simulator
fclose(u)
delete(u)