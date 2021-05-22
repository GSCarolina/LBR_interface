function [lambda, xout] = getPotentialRatio(xCenter,xWall,x,delta)
% Function that calculates the potential ratio of a point towards a wall
% [lambda] = getPotentialRatio(xCenter,xWall,x,delta)
% - xCenter = centroid location vector [x y z]
% - xWall = virtual wall closest point vector [x y z] 
% - x = point [x y z]
% - delta = distance offset (0,1) close to a wall, being 0 allowing to reach
% the virtual constraint and 1 not allowing any displacement at all
% - lambda = potential ratio from 0 to 1

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

lambda = 0.5*eta*((1/rho)-(1/rho0))^2;
xout = x;

% Rebounce point if out of limits 
if (rho<=delta*rho0 || (rho1>=rho0))
    %disp('Generating trajectory...');
%end
    %lambda = 1;
    
    if(length(x)==1) % 1D
        xout = xCenter(1) + delta*rho0;
    end
    
    if(length(x)>=2) %2 and 3D
    % Calculate angle between center and x
    %cosC = (x(1)-xCenter(1))/sqrt((x(1)-xCenter(1))^2 + (x(2)-xCenter(2))^2);
    %senC = (x(2)-xCenter(2))/sqrt((x(1)-xCenter(1))^2 + (x(2)-xCenter(2))^2);
    %C = acos(cosC);
    
    % Calculate angle between center and vc
    if(xWall(1) == xCenter(1) && xWall(2) == xCenter(2))
        cosC = 0;
        senC = 0;
    else
        cosC = (xWall(1)-xCenter(1))/sqrt((xWall(1)-xCenter(1))^2 + (xWall(2)-xCenter(2))^2);
        senC = (xWall(2)-xCenter(2))/sqrt((xWall(1)-xCenter(1))^2 + (xWall(2)-xCenter(2))^2);
    end
    
    
    xout(1) = xCenter(1) + (1-delta)*rho0*cosC;
    xout(2) = xCenter(2) + (1-delta)*rho0*senC;
    end
    
    if(length(x)==3) % 3D
        senA = (xWall(3)-xCenter(3))/sqrt((xWall(1)-xCenter(1))^2 + (xWall(3)-xCenter(3))^2);
        %A = acos(cosA);
        xout(3) = xCenter(3) + (1-delta)*rho0*senA;
    end
    
end




end

