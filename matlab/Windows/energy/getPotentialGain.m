function [g_cs] = getPotentialGain(xCenter,xWall,x,delta,Gmax,Gmin)
% Function that calculates the potential ratio of a point towards a wall
% [uo] = getPotentialRatio(xCenter,xWall,x,delta)
% - xCenter = centroid location vector [x y z]
% - xWall = virtual wall closest point vector [x y z] 
% - x = point [x y z]
% - delta = distance offset (0,1) close to a wall, being 0 allowing to reach
% the virtual constraint and 1 not allowing any displacement at all
% - uo = potential ratio from 0 to 1

% Calculate distances 
xtemp_max = 0; xtemp = 0; xmax_acc = 0; x_acc = 0; x1_acc = 0;
for i=1:length(xCenter)
    % rho0
    xtemp_max = (xWall(i)-xCenter(i))^2;
    xmax_acc = xmax_acc + xtemp_max;    
    % rho
    xtemp = (xWall(i)-x(i))^2;
    x_acc = x_acc + xtemp;
    
    % rho1
    xtemp = (x(i)-xCenter(i))^2;
    x1_acc = x1_acc + xtemp;
end

% Calculate parameters
rho0 = sqrt(xmax_acc);
rho1 = sqrt(x1_acc);
rho = sqrt(x_acc);
eta = 2*(delta^2)*(rho0^2)/(1+(delta^2)-2*delta);

lambda = 0;

% Calculate ratio
if (rho<=delta*rho0 || (rho1>=rho0))   
    lambda = 1;
else
    lambda = 0.5*eta*((1/rho)-(1/rho0))^2;
end

if(lambda < 0)
    lambda=0;
end

if(lambda > 1)
    lambda = 1;
end

g_cs = (Gmax-Gmin)*lambda + Gmin;

end

