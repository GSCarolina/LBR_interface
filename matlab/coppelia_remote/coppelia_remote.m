% On the simulation scrSipt 
% simRemoteApi.start(19999)
disp('Program started');
% sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID<=-1)
    % connection failed
    disp('Failed to connect to the remote API server');
else
    % connection successful
    disp('Connected to remote API server');
      
    % Init()
    [res,objs]=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking);
    pause(2)
    hJoint = zeros(1,7); hForce = 0;
    tJoint = zeros(1,7);
    
    disp('Retrieving joint and force handlers...');
    for i=1:7
        [res,hJoint(i)]=sim.simxGetObjectHandle(clientID,['LBR_iiwa_14_R820_joint' num2str(i)],sim.simx_opmode_blocking);
    end
    [res,hForce]=sim.simxGetObjectHandle(clientID,'LBR_iiwa_14_R820_connection',sim.simx_opmode_blocking);

    % Sensors
    disp('Reading joint values');
    
    % Try to read 200 times
    for k =1:200
        % Read joint forces
        for i=1:7
            tJoint(i)=sim.simxGetJointForce(clientID, hJoint(i), sim.simx_opmode_blocking);
            
            if(tJoint(i)~=0)
                displ('Something moved')
            end
        end
        % read force sensor
        sim.simxReadForceSensor(clientID, hForce, sim.simx_opmode_blocking);
        pause(1)
    end

    tJoint
    disp('Reading force values');
    
    
    % set actuators
    % syscall_act()

end