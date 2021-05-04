%% Mimic the simulation robot according to the real one
%% Connect to the real robot
u = udp('172.31.1.147',30007);
fopen(u)
nrJoints = 7;
%% Start the ROS Master
rosshutdown
rosinit
% Tranform the incoming data from the real robot into ROS topics
[jointPub,jointMsg] = rospublisher('/realQ');
[torquePub,torqueMsg] = rospublisher('/realT');

[posPub,posMsg] = rospublisher('/realPEE');
[forPub,forMsg] = rospublisher('/realFEE');

%% Main loop
%for k=0:5000
k=0;
while(true)
    % Receive and decode data from real robot
    message = [num2str(k) ';STATUS;'];
    fwrite(u,message)
    A = fread(u, 50);
    dataDouble = (str2num(char(A')));
    posJointsRad = dataDouble(1:7,1)';
    torJoints = dataDouble(8:14,1)';
    
    posCartesian = dataDouble(15:20,1)';
    forCartesian = dataDouble(21:26,1)';
   
    % Publish joints position to simulator and the Matlab in Windows
    qMsg = '';
    tMsg = '';
    for i=1:nrJoints
        qMsg = [qMsg (num2str(posJointsRad(i))) ';' ]; 
        tMsg = [tMsg (num2str(torJoints(i))) ';' ]; 
    end
    jointMsg.Data = qMsg;
    send(jointPub,jointMsg); 
    torqueMsg.Data = tMsg;
    send(torquePub,torqueMsg); 
   
    % Publish cartesian position to simulator and the Matlab in Windows
    qMsg = '';
    tMsg = '';
    for i=1:6
        qMsg = [qMsg (num2str(posCartesian(i))) ';' ]; 
        tMsg = [tMsg (num2str(forCartesian(i))) ';' ]; 
    end
    posMsg.Data = qMsg;    
    send(posPub,posMsg); 
    forMsg.Data = tMsg;
    send(forPub,forMsg); 
    
    pause(0.01);
    k=k+1;
end

%%
fclose(u)
delete(u)
