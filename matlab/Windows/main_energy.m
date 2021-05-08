%------------ MAIN CODE FOR ENERGY -------------

%% ------------ POTENTIAL ENERGY
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
%% Generate a Set of Cartesian-Space Waypoints
% Structure of WayPoints matrix: 
%[x0 y0 z0;
% x1 y1 z0; ...
% xn yn zn];
disp('Generating trajectory...');
Waypoints = [ 0.50  0.05  0.07; 
              0.40  0.15  0.10; 
              0.35  0.30  0.10 ]; 
%              0.30  0.40  0.10 ; 
%              0.20  0.45  0.10; 
%              0.20  0.45  0.10];
n = 10;
Path = GeneratePath(n,Waypoints,'with');
size_Path = size(Path);

%% Virtual wall
% example plane
% Use fourth input for color scale.
patch([1 -1 -1 1], [1 1 -1 -1], [0 0 0 0], [1 1 -1 -1])  
[x y] = meshgrid(-1:0.1:1); % Generate x and y data
z = zeros(size(x, 1)); % Generate z data
surf(x, y, z) % Plot the surface

% example with matrices
A = [104,122,111];
B = [253,122,153];
C = [104,124,111];
normal=normalize( cross(A-B,A-C),'norm');   %calculate plane parameters
P=dot(A,normal);
[X,Y,Z]=meshgrid(1:128,1:128,(1:27)-110);

[x,y]=meshgrid(linspace(1,128,512));        %compute sample locations in plane
z=(normal(1)*x+normal(2)*y-P)./normal(3);

surf(x, y, z) % Plot the surface

% definition?

%% Energy from a point
% using the electric charge:
% qE = Fe, where q = charge, E = distance?, F = force
% the electric potential is 0 on the end-effector
% the potential is Emax on the wall
pX = 0.3;
pWall = 0.5;
pEE = 0.0;
th = 0.1;

% Using already Emax as lambda:
Emax = 1;
Emin = 0;

% E = (mx + n)*q?;
% when E -> 1 = m (pWall-th) + n;
% n = m (Wall-pEE-th) - 1;
%
% when E -> Emin = m pEE + n;
% 0 = m pEE + n; n =- m pEE;
% 
% m (pWall-pEE-th) - m pEE = 1;
% Condition: pWall - th - pEE != 0

    %m = 1 / (pWall - pEE - th);

m = 1/(pWall -th -pEE);
if pEE>pWall
    m = 1/(pWall +th -pEE);
end
n = -m*pEE;
  
% Energy on that point
q = 1;

if E>=1
    E = 1;
    % rebounce pX inside the wall
    if pWall >= pEE
        pX = pWall - th;
    else
        pX = pWall + th;
    end
end


% This is for a fixed point-wall
Waypoints = [ 0.50  0.05  0.07; 
              0.40  0.15  0.10; 
              0.35  0.30  0.10 ; 
              0.30  0.40  0.10 ; 
              0.20  0.45  0.10; 
              0.20  0.45  0.10];  
pWall = [0.5, 0.5 , 0.5];
pEE = [0.0, 0.0 , 0.0];
th = 0.1;

pOut = zeros(length(Waypoints),3);
energy = zeros(length(Waypoints),3);

for j=1:length(Waypoints)
    [energy(j,:),pOut(j,:)] = rebouncePoints(pEE,pWall,Waypoints(j,:),th);
end

% Now necessary to define here a wall as a plane and calculate the energy
% through its vector

%% Wall on the Z axis
Waypoints = [ 0.50  0.05  0.07; 
              0.40  0.15  0.50; 
              0.35  0.30  0.50 ; 
              0.30  0.40  0.0 ; 
              0.20  0.45  0.20; 
              0.20  0.45  0.50];
pWallZ = 0.3; 
% inside
pWallDistance = 0;
% outside
pWallDistance = 1;

pWall = [pWallDistance*ones(length(Waypoints),2) pWallZ*ones(length(Waypoints),1)];

pEE_inside = [0.0, 0.0 , 0.0];
pEE_outside = [1.0, 1.0 , 1.0];
th = 0.05;

correctedWay = zeros(length(Waypoints),3);
energy = zeros(length(Waypoints),3);

for j=1:length(Waypoints)
    [energyWay(j,:),correctedWay(j,:)] = rebouncePoints(pEE_inside,pWall(j,:),Waypoints(j,:),th);
end

for j=1:length(Waypoints)
    [energyWay(j,:),correctedWay(j,:)] = rebouncePoints(pEE_outside,pWall(j,:),Waypoints(j,:),th);
end

% Get smooth trajectory
NSamples = 10; % amount of intermediate points
config = 'without';
givenPath = genPath(NSamples,Waypoints,config);
%correctedPath = genPath(NSamples,correctedWay,config);
%inside
for j=1:length(givenPath)
    [energyGiven(j,:),correctedPath(j,:)] = rebouncePoints(pEE_inside,pWall(1,:),givenPath(j,:),th);
end
%outside
for j=1:length(givenPath)
    [energyGiven(j,:),correctedPath(j,:)] = rebouncePoints(pEE_outside,pWall(1,:),givenPath(j,:),th);
end
% Plot waypoints
figure
% 3D trajectories
subplot(1,2,1)
grid on
hold on
plot3(Waypoints(:,1),Waypoints(:,2),Waypoints(:,3),'or','LineWidth', 2);
plot3(givenPath(:,1),givenPath(:,2),givenPath(:,3),'-r','LineWidth', 2);

plotObj = plot3(correctedWay(:,1),correctedWay(:,2),correctedWay(:,3),'o','LineWidth', 2);
plotObj.Color = 'k'; %'#7E2F8E';
plotObj = plot3(correctedPath(:,1),correctedPath(:,2),correctedPath(:,3),'-','LineWidth', 2);
plotObj.Color = 'k'; %'#7E2F8E';


title('Planned Path and correction')
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
axis([min(Waypoints(:,1))-0.1 max(Waypoints(:,1))+0.1 min(Waypoints(:,2))-0.1 max(Waypoints(:,2))+0.1 min(Waypoints(:,3))-0.1 max(Waypoints(:,3))+0.1])
% Add the wall
% example plane
% Use fourth input for color scale.
%patch([1 -1 -1 1], [1 1 -1 -1], [0 0 0 0], [1 1 -1 -1])  
[x y] = meshgrid(-1:0.1:1); % Generate x and y data
z = pWallZ * ones(size(x, 1)); % Generate z data
surfWall = surf(x, y, z,'FaceAlpha',0.3); % Plot the surface
surfWall.EdgeColor = 'none';

legend('Waypoints','Given path', 'Corrected waypoints', 'Corrected path', 'Wall')

% Energy
subplot(1,2,2)
grid on
hold on

energy_path = zeros(length(givenPath),3);
dummy_path = zeros(length(givenPath),3);

dataY = energyGiven(:,3);
plot([0:length(givenPath)-1],dataY,'-r')

energy_path = zeros(length(correctedPath),3);
dummy_path = zeros(length(correctedPath),3);

for j=1:length(energy_path)
    [energy_path(j,:),dummy_path(j,:)] = rebouncePoints(pEE_outside,pWall(1,:),correctedPath(j,:),th);
end
dataX = [0:length(correctedPath)-1];
dataY = energy_path(:,3);
plot(dataX,dataY,'-k')

title('Planned path energy')
xlabel('Path point [nr]')
ylabel('Energy [nominal]')
legend('Given path', 'Corrected path')
xlim([dataX(1) dataX(end)])

%% ------------ KINETIC ENERGY
Waypoints = [ 0.05  0.05  0.05; 
              0.40  0.15  0.10; 
              0.35  0.30  0.10 ; 
              0.30  0.40  0.10 ; 
              0.20  0.45  0.10; 
              0.20  0.45  0.10];  
xWall = [0.5, 0.5 ,y 0.5];
xCenter = Waypoints(1,:);
x_old = xCenter;
x=xCenter;

deltaX = 0.01;
dX = [xCenter(1):deltaX:xWall(1); xCenter(2):deltaX:xWall(2); xCenter(3):deltaX:xWall(3)];
dX = dX';

Ek= [];
for(j=1:length(dX))
    energy = getKineticEnergy(xCenter,xWall,x,dX(j,:));
    Ek = [Ek; energy];
end
Ek = Ek';

dX_unit = [];
for (j=1:length(dX))
    velocity = dX(j,1)^2 + dX(j,2)^2 + dX(j,3)^2;
    dX_unit = [dX_unit; (sqrt(velocity))];
end
dX_unit = dX_unit';

% absolute values
plot(dX_unit, Ek);


% relative values
max_velocity = (xCenter(1)-xWall(1))^2 + (xCenter(2)-xWall(2))^2 + (xCenter(3)-xWall(3))^2;
max_velocity = sqrt(max_velocity);

plot((1/max_velocity)*dX_unit, Ek);

%% Defining a non-dimensional energy field
xWall_1 = 0.05*ones(1,3); 
xWall_2 = 0.2*ones(1,3); 
xWall_3 = 0.8*ones(1,3); 

xCenter = [0.0 0.0 0.0];
xEE = xCenter;
th = 0.001;

deltaX = 0.001;

% Potential energy for different scenarios
U1 = [];
dX1 = [xCenter(1):deltaX:xWall_1(1); xCenter(2):deltaX:xWall_1(2); xCenter(3):deltaX:xWall_1(3)];
dX1 = dX1';
[amount dummy] = size(dX1);
for j=1:length(dX1)
    [energy,correctedWay] = rebouncePoints(xCenter,xWall_1,dX1(j,:),th);
    U1 = [U1; energy];
end

U2 = [];
dX2 = [xCenter(1):deltaX:xWall_2(1); xCenter(2):deltaX:xWall_2(2); xCenter(3):deltaX:xWall_2(3)];
dX2 = dX2';
for j=1:length(dX2)
    [energy,correctedWay] = rebouncePoints(xCenter,xWall_2,dX2(j,:),th);
    U2 = [U2; energy];
end

U3 = [];
dX3 = [xCenter(1):deltaX:xWall_3(1); xCenter(2):deltaX:xWall_3(2); xCenter(3):deltaX:xWall_3(3)];
dX3 = dX3';
for j=1:length(dX3)
    [energy,correctedWay] = rebouncePoints(xCenter,xWall_3,dX3(j,:),th);
    U3 = [U3; energy];
end

% calculating gains
Kaxis_min = 200; Kaxis_max = 5000;

K1 = [];
for j=1:length(U1(:,3))
    k = setParameter(U1(j,3),Kaxis_max,Kaxis_min);
    K1 = [K1; k];
end
K2 = [];
for j=1:length(U2(:,3))
    k = setParameter(U2(j,3),Kaxis_max,Kaxis_min);
    K2 = [K2; k];
end
K3 = [];
for j=1:length(U3(:,3))
    k = setParameter(U3(j,3),Kaxis_max,Kaxis_min);
    K3 = [K3; k];
end

% plotting energy and gains
figure
subplot(2,1,1)
grid on
hold on
title('Potential energy with three different obstacles')
plot([dX1(:,3)' 1.1*dX3(end)] ,[U1(:,3)' U1(end)],'-b',dX1(end),U1(end),'ob');
plot([dX2(:,3)' 1.1*dX3(end)] ,[U2(:,3)' U2(end)],'-r',dX2(end),U2(end),'or');
plot([dX3(:,3)' 1.1*dX3(end)] ,[U3(:,3)' U3(end)],'-g',dX3(end),U3(end),'og');
xlim([dX1(1) dX3(end)]);
xlabel('Xee position [m]')
ylim([0 1.1]);
ylabel('Potential energy [unit]')
legend('U1','xWall 1', 'U2','xWall 2', 'U3','xWall 3')
hold off

subplot(2,1,2)
grid on
hold on
title('Stiffness gain with three different obstacles')
plot([dX1(:,3)' 1.1*dX3(end)] ,[K1' K1(end)],'-b',dX1(end),K1(end),'ob');
plot([dX2(:,3)' 1.1*dX3(end)] ,[K2' K2(end)],'-r',dX2(end),K2(end),'or');
plot([dX3(:,3)' 1.1*dX3(end)] ,[K3' K3(end)],'-g',dX3(end),K3(end),'og')
xlim([dX1(1) dX3(end)]);
xlabel('Xee position [m]')
ylim([0 1.1*5000]);
ylabel('C-axis gain [k]')
legend('K1','xWall 1', 'K2','xWall 2', 'K3','xWall 3')
hold off

%% ------------ ENERGY APPLICATIONS

%% Using the energy to get the spring stiffness and damping parameters 
deltaE = 0.001;
Ek = [0:deltaE:1];
U = [0:deltaE:1];

% Mapping the Stiffness for C-axis
Kaxis = zeros(1,length(U));
Kaxis_min = 200; Kaxis_max = 5000;

for i=1:length(Kaxis)
    Kaxis(i) = setParameter(U(i),Kaxis_max,Kaxis_min);
end

% Mapping the Stiffness for C-angles
Kangles = zeros(1,length(U));
Kangles_min = 10; Kangles_max = 300;

for i=1:length(Kangles)
    Kangles(i) = setParameter(U(i),Kangles_max,Kangles_min);
end

% Mapping the Damping
D = zeros(1,length(Ek));
Dmin = 0.1; Dmax = 1;
for i=1:length(D)
    D(i) = setParameter(Ek(i),Dmax,Dmin);
end

%% Comparing java to matlab
% load Java data
data = load("outJava.txt");
Kaxis_java = data(:,1)';
Kangles_java = data(:,2)';
D_java = data(:,3)';

% plot and compare
for k=1:3
   switch k
       case 1
           figure('Name','Stiffness axis')
           dataX = U;
           dataY1 = Kaxis;
           dataY2 = Kaxis_java;
       case 2
           figure('Name','Stiffness angles')
           dataX = U;
           dataY1 = Kangles;
           dataY2 = Kangles_java;
       case 3
           figure('Name','Damping axis')
           dataX = Ek;
           dataY1 = D;
           dataY2 = D_java;
   end
           
   subplot(1,2,1)
   title('Parameters value')
   grid on
   hold on
   plot(dataX, dataY1, dataX, dataY2);
   hold off
   
   diffY = dataY1 - dataY2;
   subplot(1,2,2)
   title('Absolute difference bettween values')
   grid on
   hold on
   plot(dataX, diffY);
   hold off
end
% conlcusion: the java double values are OK and close enough to the Matlab
% values

