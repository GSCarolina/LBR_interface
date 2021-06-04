%------------ MAIN CODE FOR VIRTUAL WALLS -------------
%% Adding dependencies
%clear
%clc
%Change this command to your repository address. 
folder_model = ['C:\Users\Carolina\Desktop\net_shared\LBR_interface\matlab\Windows\model_urdf'];
folder_functions = ['C:\Users\Carolina\Desktop\net_shared\LBR_interface\matlab\Windows\'];
addpath(folder_model);
addpath(folder_functions);

% Add source directories
fcn_path = genpath('vwall');
addpath(fcn_path);

%% Defining a virtual wall
% Standard Plane plane:
figure 
hold on


[X,Y] = meshgrid(3:0.5:13,3:0.5:13);
[sizeM sizeN] = size(X);
Z = 80*ones(sizeM,sizeN);
flatPlane = surf(X,Y,Z,'FaceAlpha',0.7, 'FaceColor','#0072BD')

% Turned plane:
[X,Y] = meshgrid(0:0.5:10,0:0.5:10);
[sizeM sizeN] = size(X);
m = 15;
n = 0.3;
xcenter = 0; ycenter = 0;
Z = zeros(sizeM,sizeN);

for i=1:sizeM
    for j=1:sizeN
        Z(i,j) = m*(X(i,j)-xcenter) + n;
    end
end
hold on
turnPlane = surf(X,Y,Z,'FaceAlpha',0.7, 'FaceColor','#0072BD')
%'FaceColor' = select color in a hexadecimal value
xlabel('x [m]')
ylabel('y [m]')
zlabel('Potential energy')

%% Only in 2 D (not going well)
% virtual wall 1: y1 = m1*x + n1
x1 = [0:0.001:10];
m1 = 3; n1 = 1;
y1 = m1*x1 + n1*ones(1,length(x1));

% virtual wall 2: y2 = m2*x + n2
x2 = [3:0.001:13];
m2 = 0.5; n2 = 20;
y2 = m2*x1 + n2*ones(1,length(x2));

% tangent circle
r = 15;
y11 = m1*x1 + (n1-r)*ones(1,length(x1));
y22 = m2*x2 + (n2-r)*ones(1,length(x2));
% circle center
 yc = m2*xc + (n2-r);
xcircle = [(xc-r):0.001:(xc+r)];
ycPlus = zeros(1,length(xcircle));
ycMinus = zeros(1,length(xcircle));
for i=1:length(xcircle)
    ycPlus(i) = yc + sqrt(r^2-(xcircle(i)-xc)^2);
    ycMinus(i) = yc - sqrt(r^2-(xcircle(i)-xc)^2);
end

figure
hold on
plot(x1,y1,'-b',x2,y2,'-b')
plot(x1,y11,'-r',x2,y22,'-r',xc,yc,'or')
plot(xcircle,ycPlus,'-r',xcircle,ycMinus,'-r');

%%  dont use this
% virtual wall 1: y1 = m1*x + n1
x1 = [0:0.01:10];
m1 = 3; n1 = 1;
y1 = m1*x1 + n1*ones(1,length(x1));

% virtual wall 2: y2 = m2*x + n2
x2 = [3:0.01:13];
m2 = 0.5; n2 = 20;
y2 = m2*x1 + n2*ones(1,length(x2));

diffY = zeros(length(y1),length(y1));
pos = [];
for i=1:length(diffY)
    for j=1:length(diffY)
        % calculate difference between points
        diffY(i,j) = y1(i)-y2(j);
        % if the point intersects there, save it
        if (diffY(i,j)<=0.001)
            pos = [pos; i j];
        end
    end
    %i; % just to make sure the loop is running properly
end
plot(pos(:,1)', pos(:,2)')
% save('pos_test.mat','pos','diffY','y1','y2','x1','x2')

%% Using splines 
deltaX = 0.001;
x1_init = 0; x1_end = 10;
x1 = [x1_init:deltaX:x1_end];
m1 = 3; n1 = 1;
y1 = m1*x1 + n1*ones(1,length(x1));

% virtual wall 2: y2 = m2*x + n2
x2_init = 3; x2_end = 13;
x2 = [x2_init:deltaX:x2_end];
m2 = 0.5; n2 = 20;
y2 = m2*x2 + n2*ones(1,length(x2));

xc = (n2-n1)/(m1-m2);
yc2 = m1*xc + n1;
yc1 = m2*xc + n2;

figure
hold on
plot(x1,y1,x2,y2,xc,yc1,'or')

% set a radio and save all the points from r to 2 r?
r = 3;
% find the center, c+r and c+2r positions
posY1_c = 1; posY1_init = 1; posY1_end = 1;
for i=1:length(y1)
    if (abs(y1(i)-yc1)<=0.0001)
        posY1_c = i;
    end
    if (abs(y1(i)-(yc1-r))<= 0.0001)
        posY1_end = i
    end
    if (abs(y1(i)-(yc1-2*r))<= 0.0001)
        posY1_init = i
    end
    if(i == length(y1) && posY1_end == 1)
        posY1_end = i
    end    
end
posY2_c = 1; posY2_init = 1; posY2_end = 1;
for i=1:length(y2)
    if (abs(y2(i)-yc2)<=0.0001)
        posY2_c = i;
    end
    if (abs(y2(i)-(yc2+r))<= 0.0001 && i >= posY2_c)
        posY2_init = i;
    end
    if (abs(y2(i)-(yc2+2*r))<= 0.0001 && i>= posY2_c)
        posY2_end = i;
    end
    if(i == length(y2) && posY2_end == 1)
        posY2_end = i;
    end
end

%Design smooth path
waypointsX = [x1(posY1_init:posY1_end) x2(posY2_init:posY2_end)];
waypointsY = [y1(posY1_init:posY1_end) y2(posY2_init:posY2_end)];
waypoints = [waypointsX' waypointsY'];
plot(x1(posY1_init:posY1_end),y1(posY1_init:posY1_end),'-b')
hold on
plot(x2(posY2_init:posY2_end),y2(posY2_init:posY2_end),'-r')

NSamples = 10;
path  = genCorner(NSamples,waypoints);

plot(path(:,1),path(:,2))
plot(waypoints(:,1),waypoints(:,2))

%% testing
Waypoints = [ 0.50  0.05  0.07; 
              0.40  0.15  0.50; 
              0.35  0.30  0.50 ; 
              0.30  0.40  0.0 ; 
              0.20  0.45  0.20; 
              0.20  0.45  0.50];
NSamples = 10;
path  = genCorner(NSamples,Waypoints);       
plot(Waypoints(:,1),Waypoints(:,2),'-b')
plot(path(:,1),path(:,2),'-r')

%% FINDING THE CLOSEST POINT TO A VIRTUAL CONSTRAINT
xWall = [10; 10; 0.3];
xCenter = [0; 0; 0];
x1 = [3; 3; 0.1];
x2 = [-3; 2; 0.1];
x3 = [-1; -4; 0.1];
x4 = [0.2; -3; 0.1];
vTube_R = 4.5; %m

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

% Calculate angle between center and vc (I)
if(x1(1) == x1(1) && x1(2) == xCenter(2))
    cosC1 = 0;
    senC1 = 0;
else
    cosC1 = (x1(1)-xCenter(1))/sqrt((x1(1)-xCenter(1))^2 + (x1(2)-xCenter(2))^2);
    senC2 = (x1(2)-xCenter(2))/sqrt((x1(1)-xCenter(1))^2 + (x1(2)-xCenter(2))^2);
end
senA1 = (x1(3)-xCenter(3))/sqrt((x1(1)-xCenter(1))^2 + (x1(3)-xCenter(3))^2);
vTube1 = [vTube_R*cosC1; vTube_R*senC2; vTube_R*senA1];

% Calculate angle between center and vc (II)
if(x2(1) == x2(1) && x2(2) == xCenter(2))
    cosC2 = 0;
    senC2 = 0;
else
    cosC2 = (x2(1)-xCenter(1))/sqrt((x2(1)-xCenter(1))^2 + (x2(2)-xCenter(2))^2);
    senC2 = (x2(2)-xCenter(2))/sqrt((x2(1)-xCenter(1))^2 + (x2(2)-xCenter(2))^2);
end
senA2 = (x2(3)-xCenter(3))/sqrt((x2(1)-xCenter(1))^2 + (x2(3)-xCenter(3))^2);
vTube2 = [vTube_R*cosC2; vTube_R*senC2; vTube_R*senA2];

% Calculate angle between center and vc (III)
if(x3(1) == x3(1) && x3(2) == xCenter(2))
    cosC3 = 0;
    senC3 = 0;
else
    cosC3 = (x3(1)-xCenter(1))/sqrt((x3(1)-xCenter(1))^2 + (x3(2)-xCenter(2))^2);
    senC3 = (x3(2)-xCenter(2))/sqrt((x3(1)-xCenter(1))^2 + (x3(2)-xCenter(2))^2);
end
senA3 = (x3(3)-xCenter(3))/sqrt((x3(1)-xCenter(1))^2 + (x3(3)-xCenter(3))^2);
vTube3 = [vTube_R*cosC3; vTube_R*senC3; vTube_R*senA3];

% Calculate angle between center and vc (IV)
if(x4(1) == x4(1) && x4(2) == xCenter(2))
    cosC4 = 0;
    senC4 = 0;
else
    cosC4 = (x4(1)-xCenter(1))/sqrt((x4(1)-xCenter(1))^2 + (x4(2)-xCenter(2))^2);
    senC4 = (x4(2)-xCenter(2))/sqrt((x4(1)-xCenter(1))^2 + (x4(2)-xCenter(2))^2);
end
senA4 = (x4(3)-xCenter(3))/sqrt((x4(1)-xCenter(1))^2 + (x4(3)-xCenter(3))^2);
vTube4 = [vTube_R*cosC4; vTube_R*senC4; vTube_R*senA4];

%vC = 
% tube definitions
x_plot = [(xCenter(1)-vTube_R):0.01:(xCenter(2)+vTube_R)];
y_plot1 = zeros(1,length(x_plot)); y_plot2 =  y_plot1;

for(k=1:length(y_plot1))
    y_plot1(k) = sqrt(vTube_R^2 - (x_plot(k)-xCenter(1))^2 );
    y_plot2(k) = -y_plot1(k);
end

figure
grid on
hold on
plot(x_plot,y_plot1,'-b',x_plot,y_plot2,'-b')

plot(xCenter(1),xCenter(2),'*k')
plot(x1(1),x1(2), '*r',vTube1(1),vTube1(2),'or')
plot(x2(1),x2(2), '*g',vTube2(1),vTube2(2),'og')
plot(x3(1),x3(2), '*c',vTube3(1),vTube3(2),'oc')
plot(x4(1),x4(2), '*m',vTube4(1),vTube4(2),'om')


%% Checking that the Java files work
dataJava = load('javaTube.txt');
xCenter = dataJava(1,1:2);
xPos = dataJava(:,3:4);
xTube = dataJava(:,5:6);

% showing stuff
figure
grid on
hold on
plot(xCenter(1),xCenter(2),'*k',xPos(:,1),xPos(:,2),'-b',xTube(:,1),xTube(:,2),'-r')

% calculating if the circle is correct
mX = []; mTube = [];

for k=1:length(xCenter(:,1))
   mX = [mX; (xPos(1)-xCenter(1))/(xPos(2)-xCenter(2))]; 
   mTube = [mTube; (xTube(1)-xCenter(1))/(xTube(2)-xCenter(2))]; 
   
end

diff = mX - mTube;

max(diff)
% if it shows 0 it means that it is fine

