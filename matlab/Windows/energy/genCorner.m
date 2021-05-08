function path  = genCorner(NSamples,waypoints)
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

n = round(NSamples);
sizeW = size(waypoints);
%Check if points are reachable

%Design smooth path
waypoints_fcnC = (waypoints(:,1:2))';
[pointsC, t1] = fnplt(cscvn(waypoints_fcnC));

path_temp=zeros(sizeW(1)*NSamples,2);

Delta = length(t1)/(sizeW(1)*NSamples);
for i=1:(length(pointsC))
    index = round(1+(i-1)*Delta);
    path_temp(i,1) = pointsC(1,index);
    path_temp(i,2) = pointsC(2,index);
    i
end 
    path = path_temp;

end