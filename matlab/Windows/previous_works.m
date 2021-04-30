%% Plotting others people formulas for the report XD
%% Original Warren potential function
% Uin = Umax ( 1 - (Rin/Rmax)) + Uoffset
% constants
Umax = 100;
Rmax = 20;
Uoffset = 5;
% Mapping values
% deltaR = 0.1;
% Rin = [0:deltaR:15];
% Uin = zeros(1,length(Rin));
% 
% for i=1:length(Uin)
%     Uin(i) = Umax*(1-(Rin(i)/Rmax)) + Uoffset;
% end

% On 3D
% xy points on a plane
[X,Y] = meshgrid(0:0.5:10,0:0.5:10);
[m n] = size(X);
Umax_m = Umax*ones(m,n);
Uoffset_m = Uoffset*ones(m,n);
UmaxRmax = Umax/Rmax;
xcenter = 5; ycenter = 5;
Rin_m = zeros(m,n);
for i=1:m
    for j=1:n
        Rin_m(i,j) = sqrt((X(i,j)-xcenter)^2 + (Y(i,j)-ycenter)^2);
    end
end

Uin_m = Umax_m.*ones(m,n)-UmaxRmax*Rin_m + Uoffset_m;

% plotting
figure('Name','Potential field Charles Warren')
grid on
hold on
%plot(Rin,Uin)
surf(X,Y,Uin_m)
xlabel('x [m]')
ylabel('y [m]')
zlabel('Potential energy')
title('Potential energy towards an obstacle')
hold off

%% Khatib
% Original function
% Uo = 1/2*eta*((1/rho)-(1/rho0))^2;  
% constants
eta = 40;
rho0 = 5;

delta = 0.1;
rho = [1:delta:30];

Uo = zeros(1, length(rho));

for i=1:length(Uo)
    if (rho(i)<rho0)
        Uo(i) = 0;
    else
        Uo(i) = 0.5*eta*((1/rho(i))-(1/rho0))^2;
    end  
end

figure('Name','Potential field Khatib')
grid on
hold on
plot(rho,Uo)
xlim([rho(1) rho(end)])
ylim([0.9*min(Uo) 1.1*max(Uo(1,50:end))])
xlabel('Distance to the obstacle')
ylabel('Potential energy')
title('Repulsive APF approach')
hold off