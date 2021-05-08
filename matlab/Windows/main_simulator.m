%------------ MAIN CODE FOR SIMULATOR -------------
%% Start connection with simulator
u = udp('127.0.0.1',30002); % only can be a client
fopen(u)

%% Start up variables
Xee_acc = [];
Ree_acc = [];
Fee_acc = [];
Mee_acc = [];

%% main loop
for k=0:1
    % exchange data with simulator
    message = ['Matlab socket'];
    fwrite(u,message)
    A = fread(u, 40);
    recMsg = ['Received: ' char(A')];
    %disp(recMsg);
    
    % process data
    dataDouble = (str2num(char(A')));    
    Xee = dataDouble(1:3,1)';
    Ree = dataDouble(4:6,1)';
    Fee = dataDouble(7:9,1)';
    Mee = dataDouble(10:12,1)';
    
    Xee_acc = [Xee_acc; Xee];
    Ree_acc = [Ree_acc; Ree];
    Fee_acc = [Fee_acc; Fee];
    Mee_acc = [Mee_acc; Mee];
    
    % loop settings
    pause(0.1);
end

fwrite(u,'ENDSOCKET')

%% Plot data
figure
hold on
plot3(Xee_acc(:,1),Xee_acc(:,2),Xee_acc(:,3),'-r','LineWidth', 2);
plot3(Xee_acc(:,1),Xee_acc(:,2),Xee_acc(:,3),'or','LineWidth', 2);
hold off
%% End connection with simulator
fclose(u)
delete(u)