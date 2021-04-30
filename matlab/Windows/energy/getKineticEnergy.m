function [energy] = getKineticEnergy(xCenter,xWall,x,x_old)
% Function that calculates the kinetic energy of a point
% [energy] = getKineticEnergy(xCenter,xWall,x,x_old)
% - xCenter = goal position vector [x y z]
% - xWall = virtual wall closest point vector [x y z] 
% - x = point [x y z]
% - x_old = previous point [x y z]
% - energy = vector with the potential energy in each axis [x y z]

dxmax = zeros(1,3);
dxmax_unit = 1; k = 1;

% Calculate maximum velocity:
for i=1:3
    dxmax(i) = (xCenter(i)-xWall(i))^2;   
end
dxmax_unit = sqrt(dxmax(1)+dxmax(2)+dxmax(3));

if (dxmax_unit > 0)
    k = 2/(dxmax_unit^2);
end

dx = zeros(1,3);
dx_unit = 1;
% Calculate current velocity
for i=1:3
    dx(i) = (x(i)-x_old(i))^2;   
end
dx_unit = sqrt(dx(1)+dx(2)+dx(3));

% Potential energy
energy = 0.5*k*dx_unit*dx_unit;

end

