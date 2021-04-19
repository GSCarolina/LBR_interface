function [energy,pOut] = rebouncePoints(pCenter,pWall,pX,th)
% Checks if a point is close to a wall and puts it back

pOut = zeros(1,3);
energy = zeros(1,3);
for i=1:3 %3D dimensional points
    m = 1/(pWall(i) -th -pCenter(i));
    if pCenter(i)>pWall(i)
        m = 1/(pWall(i) +th -pCenter(i));
    end
    n = -m*pCenter(i);
    
    E = m*pX(i)+n;
    
    % check boundaries
    if E<=1
        pOut(i) = pX(i);
    else
        %E = 1;
        % rebounce pX inside the wall
        if pWall(i) >= pCenter(i)
            pOut(i) = pWall(i) - th;
        else
            pOut(i) = pWall(i) + th;
        end
    end
    
%     if E<0
%         E = 0;
%     end
    
    energy(i) = E;
end
end

