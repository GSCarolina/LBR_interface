%------------ MAIN CODE FOR ENERGY (VERSION 2) -------------

%% Adding dependencies
%clear
%clc
%Change this command to your repository address. 
folder_model = ['C:\Users\Carolina\Desktop\net_shared\LBR_interface\matlab\Windows\model_urdf'];
folder_functions = ['C:\Users\Carolina\Desktop\net_shared\LBR_interface\matlab\Windows\'];
addpath(folder_model);
addpath(folder_functions);

% Add source directories
fcn_path = genpath('energy');
addpath(fcn_path);

%% ------------ POTENTIAL ENERGY
% Uo(i) = 0.5*eta*((1/rho(i))-(1/rho0))^2;

% rho0 = Rmax = abs(xCenter - xWall)
% rho  = R    = abs(x - xWall)

rho0 = 25;
delta = 0.1;
rho = [1:delta:30];

margin1 = 0.5;
eta1 = 2*(margin1^2)*(rho0^2)/(1+(margin1^2)-2*margin1);
Uo1 = zeros(1, length(rho));
Umargin1 = 0;

for i=1:length(Uo1)
    if (rho(i)>rho0)
        Uo1(i) = 0;
    elseif (rho(i)<=margin1*rho0)
        Uo1(i) = 1;
    else
        Uo1(i) = 0.5*eta1*((1/rho(i))-(1/rho0))^2;
    end  
    
    if(rho(i) == margin1*rho0)
        Umargin1 = Uo1(i);
    end
end

margin2 = 0.9;
eta2 = 2*(margin2^2)*(rho0^2)/(1+(margin2^2)-2*margin2);
Uo2 = zeros(1, length(rho));
Umargin2 = 0;

for i=1:length(Uo2)
    if (rho(i)>rho0)
        Uo2(i) = 0;
    elseif (rho(i)<=margin2*rho0)
        Uo2(i) = 1;
    else
        Uo2(i) = 0.5*eta2*((1/rho(i))-(1/rho0))^2;
    end  
    
    if(rho(i) == margin2*rho0)
        Umargin2 = Uo2(i);
    end
end

figure
grid on
hold on
plot(rho,Uo1,'-b',margin1*rho0,Umargin1,'*b')
plot(rho,Uo2,'-r',margin2*rho0,Umargin2,'*r')
xlim([rho(1) rho(end)])
%ylim([0.9*min(Uo) max(Uo(1,20:end))])
hold off

%% Testing the potential ratio and gains
xCenter =[0; 0];
xWall = [10; 10];
x1 = [0:0.001:10]; x2 = [0:0.001:10]; x = [x1' x2'];
[m,n] = size(x);

xOff1 = 0.3;
uo1 = zeros(1,length(x1));

for i=1:length(uo1)
    uo1(i) = getPotentialRatio(xCenter,xWall,x(i,:),xOff1);
end

Kmax = 500; Kmin = 200;
go1 = zeros(1,length(x1)); 

for i=1:length(go1)
    go1(i) = getPotentialGain(xCenter,xWall,x(i,:),0.3,Kmax,Kmin);
end

figure
subplot(1,2,1)
plot(x(:,1)',uo1)

subplot(1,2,2)
plot(x(:,1)',go1)

%% Testing different margins (2D)
Kmax = 500; Kmin = 200;
xCenter =[0; 0];
xWall = [10; 10];
x1 = [0:0.001:10]; x2 = [0:0.001:10]; x = [x1' x2'];

xOff1 = 0.125; xOff2 = 0.5; xOff3 = 0.75;
go1 = zeros(1,length(x1)); 
go2 = zeros(1,length(x1)); 
go3 = zeros(1,length(x1)); 

for i=1:length(go1)
    go1(i) = getPotentialGain(xCenter,xWall,x(i,:),xOff1,Kmax,Kmin);
    go2(i) = getPotentialGain(xCenter,xWall,x(i,:),xOff2,Kmax,Kmin);
    go3(i) = getPotentialGain(xCenter,xWall,x(i,:),xOff3,Kmax,Kmin);
end

plot(x(:,1)',go1,x(:,1)',go2,x(:,1)',go3)

%% Testing different margins (3D lines)
Kmax = 500; Kmin = 200;
xCenter =[0; 0];
xWall = [10; 10];
x1 = [0:0.001:10]; x2 = [0:0.001:10]; x = [x1' x2'];

xOff1 = 0.125; xOff2 = 0.5; xOff3 = 0.75;
go1 = zeros(1,length(x1)); 
go2 = zeros(1,length(x1)); 
go3 = zeros(1,length(x1)); 

for i=1:length(go1)
    go1(i) = getPotentialGain(xCenter,xWall,x(i,:),xOff1,Kmax,Kmin);
    go2(i) = getPotentialGain(xCenter,xWall,x(i,:),xOff2,Kmax,Kmin);
    go3(i) = getPotentialGain(xCenter,xWall,x(i,:),xOff3,Kmax,Kmin);
end

plot3(x(:,1)',x(:,2)',go1,x(:,1)',x(:,2)',go2,x(:,1)',x(:,2)',go3)

%% Testing different margins (3D plane)
Kmax = 500; Kmin = 200;
xCenter =[10; 10];
xWall = [5; 5];
xOff1 = 0.125; xOff2 = 0.5; xOff3 = 0.75;

[X,Y] = meshgrid(0:0.1:20,0:0.01:20);
[sizeM sizeN] = size(X);
Z1 = zeros(sizeM,sizeN);
Z2 = zeros(sizeM,sizeN);
Z3 = zeros(sizeM,sizeN);

for i=1:sizeM
    for j=1:sizeN
        Z1(i,j) = getPotentialGain(xCenter,xWall,[X(i,j) Y(i,j)],xOff1,Kmax,Kmin);
        Z2(i,j) = getPotentialGain(xCenter,xWall,[X(i,j) Y(i,j)],xOff2,Kmax,Kmin);
        Z3(i,j) = getPotentialGain(xCenter,xWall,[X(i,j) Y(i,j)],xOff3,Kmax,Kmin);
    end
end


figure
subplot(1,3,1)
grid on
hold on
turnPlane = surf(X,Y,Z1,'FaceAlpha',0.7, 'EdgeColor','none')
plot3([xCenter(1) xCenter(1)], [xCenter(2) xCenter(2)], [Kmin Kmax],'-g','LineWidth', 2)
plot3([xWall(1) xWall(1)], [xWall(2) xWall(2)], [Kmin Kmax],'-r','LineWidth', 2)
%'FaceColor' = select color in a hexadecimal value
xlabel('x [m]')
ylabel('y [m]')
zlabel('Gain')
title(['Stiffness Gain with x offset = ' num2str(xOff1)])

subplot(1,3,2)
grid on
hold on
turnPlane = surf(X,Y,Z2,'FaceAlpha',0.7, 'EdgeColor','none')
plot3([xCenter(1) xCenter(1)], [xCenter(2) xCenter(2)], [Kmin Kmax],'-g','LineWidth', 2)
plot3([xWall(1) xWall(1)], [xWall(2) xWall(2)], [Kmin Kmax],'-r','LineWidth', 2)
%'FaceColor' = select color in a hexadecimal value
xlabel('x [m]')
ylabel('y [m]')
zlabel('Gain')
title(['Stiffness Gain with x offset = ' num2str(xOff2)])

subplot(1,3,3)
grid on
hold on
turnPlane = surf(X,Y,Z3,'FaceAlpha',0.7, 'EdgeColor','none')
plot3([xCenter(1) xCenter(1)], [xCenter(2) xCenter(2)], [Kmin Kmax],'-g','LineWidth', 2)
plot3([xWall(1) xWall(1)], [xWall(2) xWall(2)], [Kmin Kmax],'-r','LineWidth', 2)
%'FaceColor' = select color in a hexadecimal value
xlabel('x [m]')
ylabel('y [m]')
zlabel('Gain')
title(['Stiffness Gain with x offset = ' num2str(xOff3)])

%%
figure
subplot(3,1,1)
grid on
hold on
turnPlane = surf(X,Y,Z1,'FaceAlpha',0.7, 'EdgeColor','none')
plot3([xCenter(1) xCenter(1)], [xCenter(2) xCenter(2)], [Kmin Kmax],'-g','LineWidth', 2)
plot3([xWall(1) xWall(1)], [xWall(2) xWall(2)], [Kmin Kmax],'-r','LineWidth', 2)
%'FaceColor' = select color in a hexadecimal value
xlabel('x [m]')
ylabel('y [m]')
zlabel('Gain')
title(['Stiffness Gain with x offset = ' num2str(xOff1)])

subplot(3,1,2)
grid on
hold on
turnPlane = surf(X,Y,Z2,'FaceAlpha',0.7, 'EdgeColor','none')
plot3([xCenter(1) xCenter(1)], [xCenter(2) xCenter(2)], [Kmin Kmax],'-g','LineWidth', 2)
plot3([xWall(1) xWall(1)], [xWall(2) xWall(2)], [Kmin Kmax],'-r','LineWidth', 2)
%'FaceColor' = select color in a hexadecimal value
xlabel('x [m]')
ylabel('y [m]')
zlabel('Gain')
title(['Stiffness Gain with x offset = ' num2str(xOff2)])

subplot(3,1,3)
grid on
hold on
turnPlane = surf(X,Y,Z3,'FaceAlpha',0.7, 'EdgeColor','none')
plot3([xCenter(1) xCenter(1)], [xCenter(2) xCenter(2)], [Kmin Kmax],'-g','LineWidth', 2)
plot3([xWall(1) xWall(1)], [xWall(2) xWall(2)], [Kmin Kmax],'-r','LineWidth', 2)
%'FaceColor' = select color in a hexadecimal value
xlabel('x [m]')
ylabel('y [m]')
zlabel('Gain')
title(['Stiffness Gain with x offset = ' num2str(xOff3)])

%% tesing rebouncing points (2D)
xCenter =[0; 0];
xWall = [10; 10];
x1 = [0:0.001:9.5]; x2 = [0:0.001:9.5]; x = [x1' x2'];

delta1 = 0.125; delta2 = 0.5; delta3 = 0.75;
go1 = []; 
go2 = zeros(1,length(x1)); 
go3 = zeros(1,length(x1)); 

for i=1:length(x1)
    [lambda, xout] = getPotentialRatio(xCenter,xWall,x(i,:),delta);
    go1 = [go1; lambda xout];
end


plot(x1,x2,go1(:,2),go1(:,3)) 
%,go1(:,1))

plot3(go1(:,2),go1(:,3),go1(:,1))

%% tesing rebouncing points (3D) -inside/outside
% Wall on the Z axis
Waypoints = [ 0.50  0.05  0.07; 
              0.40  0.15  0.50; 
              0.35  0.30  0.50 ; 
              0.30  0.40  0.0 ; 
              0.20  0.45  0.20; 
              0.20  0.45  0.50];
          
xWallZ = 0.3; 
xWall_plane = [Waypoints(:,1) Waypoints(:,2) xWallZ*ones(length(Waypoints(:,1)),1) ];

% Center
xCenterZ_in = 0.0;
xCenter_in = [Waypoints(:,1) Waypoints(:,2) xCenterZ_in*ones(length(Waypoints(:,1)),1) ];

% outside
xCenterZ_out = 1.0;
xCenter_out = [Waypoints(:,1) Waypoints(:,2) xCenterZ_out*ones(length(Waypoints(:,1)),1) ];

delta = 0.1;

fixWaypoints_in = zeros(length(Waypoints),3);
fixWaypoints_out = zeros(length(Waypoints),3);

for j=1:length(Waypoints)
    [dummyLambda,fixWaypoints_in(j,:)] = getPotentialRatio(xCenter_in(j,:),xWall_plane(j,:),Waypoints(j,:),delta);
    [dummyLambda,fixWaypoints_out(j,:)] = getPotentialRatio(xCenter_out(j,:),xWall_plane(j,:),Waypoints(j,:),delta);
end

figure
grid on
hold on
plot3(Waypoints(:,1), Waypoints(:,2), Waypoints(:,3),'-b')
plot3(fixWaypoints_in(:,1), fixWaypoints_in(:,2), fixWaypoints_in(:,3),'-r')

% figure
% grid on
% hold on
% plot([0:(length(Waypoints(:,1))-1)],lambdaWay)
% plot([0:(length(Waypoints(:,1))-1)],fixlambdaWay)

% complete path

n = 10;
recordPath = GeneratePath(n,Waypoints,'path');
fixPath_in = recordPath; fixPath_out = recordPath;

xWallZ = 0.3; 
xWall_plane = [recordPath(:,1) recordPath(:,2) xWallZ*ones(length(recordPath(:,1)),1) ];

% Center
xCenter_in = [recordPath(:,1) recordPath(:,2) xCenterZ_in*ones(length(recordPath(:,1)),1) ];
xCenter_out = [recordPath(:,1) recordPath(:,2) xCenterZ_out*ones(length(recordPath(:,1)),1) ];

lambdaWay = zeros(length(recordPath),1);
fixlambdaWay = zeros(length(recordPath),1);

for j=1:length(recordPath)
    [lambdaWay(j,1),fixPath_in(j,:)] = getPotentialRatio(xCenter_in(j,:),xWall_plane(j,:),recordPath(j,:),delta);
    %[fixlambdaWay(j,1),dummy] = getPotentialRatio(xCenter_in(j,:),xWall_plane(j,:),fixPath(j,:),delta);

    [lambdaWay(j,1),fixPath_out(j,:)] = getPotentialRatio(xCenter_out(j,:),xWall_plane(j,:),recordPath(j,:),delta);
    %[fixlambdaWay(j,1),dummy] = getPotentialRatio(xCenter_in(j,:),xWall_plane(j,:),fixPath(j,:),delta);
end

% PLOT
xLimit = [0.9*min(recordPath(:,1)) 1.1*max(recordPath(:,1))];
yLimit = [0.9*min(recordPath(:,2)) 1.1*max(recordPath(:,2))];
zLimit = [0.9*min(recordPath(:,3)) 1.1*max(recordPath(:,3))];
% adding the wall
[x y] = meshgrid(-1:0.1:1); % Generate x and y data
z = xWallZ * ones(size(x, 1)); % Generate z data


figure
subplot(1,2,1)
grid on
hold on
plot3(Waypoints(:,1), Waypoints(:,2), Waypoints(:,3),'ob')
plot3(recordPath(:,1), recordPath(:,2), recordPath(:,3),'-b')

plot3(fixWaypoints_in(:,1), fixWaypoints_in(:,2), fixWaypoints_in(:,3),'or')
plot3(fixPath_in(:,1), fixPath_in(:,2), fixPath_in(:,3),'-r')

surfWall = surf(x, y, z,'FaceAlpha',0.3,'EdgeColor', 'none'); % Plot the surface
axis([xLimit yLimit zLimit]);
legend('Waypoints','Given path', 'Corrected waypoints', 'Corrected path', 'Wall')

title('Planned Path and correction towards inside')
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')

subplot(1,2,2)
grid on
hold on
plot3(Waypoints(:,1), Waypoints(:,2), Waypoints(:,3),'ob')
plot3(recordPath(:,1), recordPath(:,2), recordPath(:,3),'-b')

plot3(fixWaypoints_out(:,1), fixWaypoints_out(:,2), fixWaypoints_out(:,3),'or')
plot3(fixPath_out(:,1), fixPath_out(:,2), fixPath_out(:,3),'-r')
surfWall = surf(x, y, z,'FaceAlpha',0.3,'EdgeColor', 'none'); % Plot the surface

axis([xLimit yLimit zLimit]);
legend('Waypoints','Given path', 'Corrected waypoints', 'Corrected path', 'Wall')

title('Planned Path and correction towards outside')
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')

%% debug code 
x = [1; 1];
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
eta = 2*(delta^2)*(rho0^2)/(1+(delta^2)-2*delta);

lambda = 0.5*eta*((1/rho)-(1/rho0))^2;

    if(length(x)==1)
        xout = xCenter(1) + delta*rho0;
    else if(length(x)>=2)
    % Calculate angle between center and x
    cosC = (x(1)-xCenter(1))/sqrt((x(1)-xCenter(1))^2 + (x(2)-xCenter(2))^2);
    senC = (x(2)-xCenter(2))/sqrt((x(1)-xCenter(1))^2 + (x(2)-xCenter(2))^2);
    %C = acos(cosC);
    
    xout(1) = xCenter(1) + delta*rho0*cosC;
    xout(2) = xCenter(2) + delta*rho0*senC;
    end
    
    if(length(x)==3)
        cosA = (x(1)-xCenter(1))/sqrt((x(1)-xCenter(1))^2 + (x(3)-xCenter(3))^2);
        %A = acos(cosA);
        xout(3) = xCenter(3) + delta*rho0*cosA;
    end
