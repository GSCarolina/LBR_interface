function [energy,xOut] = rebouncePoints(xCenter,xWall,x,Loff)
% Function that calculates the potential energy of a point towards a wall
% and in case it overpasses the wall, it pushes it back
% [energy,xOut] = rebouncePoints(xCenter,xWall,x,Loff)
% - xCenter = centroid location vector [x y z]
% - xWall = virtual wall closest point vector [x y z] 
% - x = point [x y z]
% - Loff = distance offset in m
% - energy = vector with the potential energy in each axis [x y z]
% - xOut = either original x or a modified x that is inside the safe
% workspace

xOut = zeros(1,3);
energy = zeros(1,3);
for i=1:3 %3D dimensional points
    m = 1/(xWall(i) -Loff -xCenter(i));
    if xCenter(i)>xWall(i)
        m = 1/(xWall(i) +Loff -xCenter(i));
    end
    n = -m*xCenter(i);
    
    U = m*x(i)+n;
    
    % check boundaries
    if U<=1
        xOut(i) = x(i);
    else
        U = 1;
        % rebounce x inside Loffe wall
        if xWall(i) >= xCenter(i)
            xOut(i) = xWall(i) - Loff;
        else
            xOut(i) = xWall(i) + Loff;
        end
    end
    
%     if E<0
%         U = 0;
%     end
    
    energy(i) = U;
end
end

