function path  = genPath(NSamples,waypoints,config)
%This functions generates a 3D Path from an initial position to a final
%one.
%[path] = GeneratePath_UR3(NSamples,waypoints)
%- NSamples: input int variabble. Amount of interactions from the initial 
%point to the final one.
%- Waypoints: input nx3 vector, where n is the amount of points. 
%Each point is a row and it has the components [x y z]
%- config: 'with' if there's an end-effector, 
%'without' for only manipulator.
%- Path: output Nx3 vector. Every column is in the format [x y z]. 
%> Note: maximum workspace of the UR3e model -> Sphere with r=650mm.
%        minimum workspace of the UR3e model -> Base with r=20mm.

valid = 0;
check = 3;
%% REDO THE WORKSPACE RADIOS FOR THIS
%%
% Maximum Workspace: 450mm manipulator +100 gripper +100 reach end
% -> Total: with gripper 650mm -> ^2 -> 0.36, without = 450mm = 0.2025
% Minimum Workspace: 20 inner base +50 gripper +100 reach end
% -> Total: with gripper = 170mm -> ^2 -> 0.36, without = 20mm = 0.004
rmax = 0; 
rmin = 0;
w1 = strcmp(config,'with');
w0 = strcmp(config,'without');
if w1 
rmax = 1.4225; 
rmin = 0.0289;
end
if w0
rmax = 1.2025; 
rmin = 0.004;
end
rmax = 100;
rmin = 0;
%Checking input parameters
if (rmax==0)
    error('Please specify a configuration.\n'); 
end

n = round(NSamples);
sizeW = size(waypoints);
limit2 = -1;
%Check if points are reachable

%Design smooth path
waypoints_fcnC = (waypoints(:,1:3))';
[pointsC, t1] = fnplt(cscvn(waypoints_fcnC));

path_temp=zeros(sizeW(1)*NSamples,3);

Delta = length(t1)/(sizeW(1)*NSamples);
for i=1:(sizeW(1)*NSamples)
    index = round(1+(i-1)*Delta);
    path_temp(i,1:3)=[pointsC(1,index) pointsC(2,index) pointsC(3,index)];
end 
    path = path_temp;

end