%------------ MAIN CODE FOR ENERGY -------------
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