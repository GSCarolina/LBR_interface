function [energy,pOut] = rebouncePoints2(pCenter,pWall,pX,th)
% Checks if a point is close to a wall and puts it back

    pOut = zeros(1,3);
    energy = zeros(1,3);
%for i=1:3 %3D dimensional points
    m = 1/((pWall(1) -th -pCenter(1))^2+(pWall(2)-th-pCenter(2))^2+(pWall(3)-th-pCenter(3))^3);
    if pCenter(i)>pWall(i)
        m = 1/((pWall(1)+th+pCenter(1))^2+(pWall(2)+th+pCenter(2))^2+(pWall(3)+th+pCenter(3))^3);
    end
    n = -m*pCenter(i);
    
    E = m*sqrt(pX(1)^2+pX(2)^2+pX(3)^1)+n;
    
    % check boundaries
    if E<=1
        pOut = pX;
    else
        E = 1;
        % rebounce pX inside the wall
        for i=1:3
        if pWall(i) >= pCenter(i)
            pOut(i) = pWall(i) - th;
        else
            pOut(i) = pWall(i) + th;
        end
        end
    end
    
    if E<0
        E = 0;
    end
    
    energy(i) = E;
%end
end

