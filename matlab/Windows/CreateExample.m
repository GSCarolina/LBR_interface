function CreateExample(FileName, robot, solver)
%This funtion is only to generate an example in Simulink following a
%reference
Joint_init = zeros(6,1);
% transform = getTransform(robot,Joint_init,'base_link','wrist_3_link');
% init_waypoint = (round(transform(1:3,4),6))';

% init_waypoint = 0.4570   -0.1310    0.0670
% This init_waypoint is not valid, because the arm is completely extended
% For the simulation:
Waypoints = [ 0.50  0.05  0.07; 
              0.40  0.15  0.10; 
              0.35  0.30  0.10; 
              0.30  0.40  0.10; 
              0.20  0.45  0.10; 
              0.20  0.45  0.10];
%Waypoints = [Waypoints(:,2) Waypoints(:,1) Waypoints(:,3)];
n = 10;
%Path = GeneratePath(n,Waypoints);
Path = Waypoints;
size_Path = size(Path);

effector=[-1 0 0 pi/2];
weights = [1 1 1 1 1 1];
eeName = 'gripper_link';
Joint_init = zeros(6,1);
JointPath = GenerateJointPath(effector,Waypoints,Joint_init,solver,weights,eeName);
size_JP = size(JointPath);

t_init = 0;
t_end = 5; 
t_temp = zeros(1,size_JP(1));
if (size_JP(1)==1)
    fprintf("Joint Path too small.\n");
else
    deltaT = (t_end-t_init)/(size_JP(1)-1);

    for i = 1:size_JP(1)
        t_temp(i)=t_init+deltaT*(i-1);
    end
end
Path_temp = Path(:,1:3);
SimCartesian = Path_temp';
SimJoint = JointPath';
SimTime = t_temp;
save(FileName, 'Waypoints', 'Path', 'JointPath','SimCartesian','SimJoint','SimTime');
fprintf('Example ready. Load it now into workspace. \n');
end