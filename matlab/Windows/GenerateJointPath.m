function [path] = GenerateJointPath(effector,path_XYZ,Joint_init,solver,weights,eeName)
%This functions generates a path for every joint following a path in the 
% XYZ form
%[path] = GenerateJointPath(effector,path_XYZ,Joint_init,weights)
%- effector = input 1x4 vector. Location of the end-effector.
%- path_XYZ: input Nx3 vector. It is a secuence of N points to follow.
%- Joint_init: input nx1 vector. Initial joints position. n = is the amount
% of joints of the manipulator.
%- solver: inverse kinematics component.
%- weights: input 1xn vector, where every element is the 1-treshold error.
%- eeName: manipulator end effector link name.
%- Path: output Nxn vector. N is the amount of points to follow. n is the
%amount of joints of the manipulator.

valid = 0;
check_p = [1 4];
check_e = [1 4];
nJoints_2 = size(Joint_init);
nJoints = 0;
if (nJoints_2(1) == 1)
    nJoints = nJoints_2(2);
elseif (nJoints_2(2)==1)
    nJoints = nJoints_2(1);
end
%Checking input parameters
%n = round(path_XYZ);

    if (check_e ~= size(effector))
        error('Wrong effector Position parameter.\n');
    end
    if (nJoints <=1)
        error('Wrong initial Joints position.\n');
    end

    if (check_e == size(effector))
        if (nJoints >1)
            valid = 1;
        else
            error('Wrong initial Joints parameter.\n');  
        end
    else
        error('Wrong size of effector.\n');  
    end


%If parameters are invaled, return null array
if (valid==0)
    path = zeros(nJoints,1);
else

size_path_XYZ = size(path_XYZ);
initialGuess = Joint_init;
path_temp = zeros(size_path_XYZ(1), nJoints);
efOrientation = axang2rotm(effector);

    for i = 1:size_path_XYZ(1)
        currentTaskSpaceSegment = path_XYZ(i,1:3);
        currentJointSegment = zeros(nJoints);
        pose = [efOrientation currentTaskSpaceSegment'; 0 0 0 1];
        currentJointSegment = solver(eeName,pose,weights,initialGuess);
        initialGuess = currentJointSegment;

        path_temp(i, :) = (currentJointSegment);
    end
        path = path_temp;
end
end