function [uo] = getPotentialGain(xCenter,xWall,x,xOff,Gmax,Gmin)
% Function that calculates the potential ratio of a point towards a wall
% [uo] = getPotentialRatio(xCenter,xWall,x,xOff)
% - xCenter = centroid location vector [x y z]
% - xWall = virtual wall closest point vector [x y z] 
% - x = point [x y z]
% - xOff = distance offset (0,1) close to a wall, being 0 allowing to reach
% the virtual constraint and 1 not allowing any displacement at all
% - uo = potential ratio from 0 to 1

% Calculate distances 
xtemp = 0; xmax_acc = 0; x_acc = 0;
for i=1:length(xCenter)
    % rho0
    xtemp = (xCenter(i)-xWall(i))^2;
    xmax_acc = xmax_acc + xtemp;    
    % rho
    xtemp = (x(i)-xWall(i))^2;
    x_acc = x_acc + xtemp;
end

% Calculate parameters
rho0 = sqrt(xmax_acc);
rho = sqrt(x_acc);
eta = (Gmax-Gmin)*2*(xOff^2)*(rho0^2)/(1+(xOff^2)-2*xOff);

uo = 0;

if (rho<=xOff*rho0)
    uo = Gmax;
else
    uo = Gmin + (0.5*eta*((1/rho)-(1/rho0))^2);
end   

    if(uo>Gmax)
        uo = Gmax;
    end

end

