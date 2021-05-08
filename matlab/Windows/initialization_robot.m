function [ur3e, solver] = initialization_robot(FileName)
%This function loads the robot ur3e basic elements into the workspace.
%Previously to run this function, set the location of the containing 
%folder as the current workspace.
%[ur3e, solver] = initialization_UR3w(FileName)
%- ur3e: RigidBodyTree output.
%- solver: inversekinematic output.

robot = importrobot(FileName) % Current name: 'robot_ur3e.urdf'
robot.DataFormat = 'column';
% Add basic parameters
gravityVec = [0 0 -9.80665];
robot.Gravity = gravityVec;
ik = inverseKinematics('RigidBodyTree', robot);

%Plotting the robot and its workspace in 3D
[x y z] = sphere(18);
r = 0.6;
x = x*r; y=y*r; z=z*r;
Colour = 0.6*[0.1 1 0.1];
figure
show(robot)
hold all
%surface(x,y,z,'FaceColor', 'none','EdgeColor',Colour)
title('Robot workspace')
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
%axis([-0.61 0.61 -0.61 0.61 -0.61 0.61])
hold off

%Outputs
ur3e = robot;
solver = ik;

%Send model to Simscape
%smimport(FileName);

end