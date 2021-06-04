function plotAPF(files_dir,pos,delta,delta2,figName,tSample)
% function that plots the robot recording
% plotAPF(files_dir,pos,delta, figName,tSample)
% - files_dir: directory where the recordings are located
% - pos: int position of the directory array (which recording to plot)
% - delta: non-dimensional margin (0,1) used
% - delta2: if using 2 delta for margin. 1=yes, 0=no
% - figName: string name for the figure
% - tSample: sampling time

full_name = [files_dir(pos).folder '\' files_dir(pos).name];
data=load(full_name);
xGoal = data(:,1:3);
xVC = data(:,4:6);
x = data(:,7:9);
lambda = data(:,10);
beta = data(:,11);
% Calculating rho, rho0 and rho1 (3 AXIS)
rho = zeros(1,length(lambda));
rho1 = rho; rho0 = rho; deltaRho = rho;

for j=1:length(rho)
   rho0(j) = sqrt((xGoal(j,1)-xVC(j,1))^2 + (xGoal(j,2)-xVC(j,2))^2  + (xGoal(j,3)-xVC(j,3))^2 );
   rho(j) = sqrt((x(j,1)-xVC(j,1))^2 + (x(j,2)-xVC(j,2))^2  + (x(j,3)-xVC(j,3))^2 );
   
   deltaRho(j) = delta*rho0(j);
end

figure('Name',figName)

dataX = [0:length(x(:,3))-1]*tSample;
subplot(3,1,1)
grid on
hold on
plot(dataX,xGoal(:,3),'-g',dataX,x(:,3),'-b')
plot(dataX,xVC(:,3),'-r')
title('Z axis path')
legend('x goal', 'x ee' ,'vc')
xlim([dataX(1) dataX(end)])
xlabel('Time [s]')
ylabel('Position [mm]')

subplot(3,1,3)
grid on
hold on
title('Potential ratio on path')
plot(dataX,lambda,'-k')
legend('lambda')
xlim([dataX(1) dataX(end)])
xlabel('Time [s]')
ylabel('Potential ratio [non-dim]')

subplot(3,1,2)
grid on
hold on
title('Distance between the ee-position and virtual wall')
xlim([dataX(1) dataX(end)])
xlabel('Time [s]')
ylabel('Abs.distance [mm]')
plot(dataX,rho0,'-g',dataX,rho,'-b',dataX,deltaRho,'-r')

if(delta2==1)
    delta2Rho = 2*deltaRho;
    plot(dataX,delta2Rho,'-y')
    legend('rho0','rho','d*rho','2d*rho')
else
    legend('rho0','rho','d*rho')
end

hold off


end

