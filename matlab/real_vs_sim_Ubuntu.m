%% Mimic the simulation robot according to the real one
%% Connect to the real robot
u = udp('172.31.1.147',30007);
fopen(u)
nrJoints = 7;
%% Start the ROS Master
rosinit
% Tranform the incoming data from the real robot into ROS topics
[posPub,posMsg] = rospublisher('/realQ');
[forPub,forMsg] = rospublisher('/realT');

%% Main loop
%for k=0:5000
while(true)
    % Receive and decode data from real robot
    message = [num2str(k) ';STATUS;'];
    fwrite(u,message)
    A = fread(u, 50);
    dataDouble = (str2num(char(A')));
    posJointsRad = dataDouble(1:7,1)';
    torJoints = dataDouble(8:14,1)';
    
   
    % Publish joints position to simulator and the Matlab in Windows
    qMsg = '';
    tMsg = '';
    for i=1:nrJoints
        qMsg = [qMsg (num2str(posJointsRad(i))) ';' ]; 
        tMsg = [tMsg (num2str(torJoints(i))) ';' ]; 
    end
    posMsg.Data = qMsg;
    send(posPub,posMsg); 
    forMsg.Data = tMsg;
    send(forPub,forMsg); 
   
    pause(0.001);
end


fclose(u)
delete(u)
