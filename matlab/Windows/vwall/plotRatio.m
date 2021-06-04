function [rho0,rho,deltaRho,dataY,dataX] = plotRatio(files_dir,pos,delta,figName,ratioName,tSample)
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

nlength = length(data(:,1));


% load data accordingly:
if(strcmp(ratioName,'lambda'))
    data=load(full_name);
    xGoal = data(:,1:3);
    xVC = data(:,4:6);
    x = data(:,7:9);
    lambda = data(:,10);
    beta = data(:,11);
    
    dataY = lambda;
    titleName2 = 'Potential ratio on path';
    yName2 = 'lambda [non-dim]';
    titleName1 = 'Distance between the ee-position and virtual wall';
    yName1 = 'Abs.distance [mm]';
    
else
    data=load(full_name);
    nlength = length(data(:,1)) - 1;
    xGoal = zeros(length(data(2:end,1)),3);
    xVC = data(2:end,1:3);
    x = data(2:end,4:6);
    beta = data(2:end,7);
    
    dataY = beta;
    titleName2 = 'Velocity ratio overtime';
    yName2 = 'beta [non-dim]';
    titleName1 = 'Velocity limit values';
    yName1 = 'Abs.velocity [mm/s]';
end

% Calculating rho, rho0 and rho1 (3 AXIS)
rho = zeros(1,nlength);
rho1 = rho; rho0 = rho; deltaRho = rho;

for j=1:length(rho)
   rho0(j) = sqrt((xGoal(j,1)-xVC(j,1))^2 + (xGoal(j,2)-xVC(j,2))^2  + (xGoal(j,3)-xVC(j,3))^2 );
   rho(j) = sqrt((x(j,1)-xVC(j,1))^2 + (x(j,2)-xVC(j,2))^2  + (x(j,3)-xVC(j,3))^2 );  
   deltaRho(j) = delta*rho0(j);
end

figure('Name',figName)

dataX = [0:(nlength-1)]*tSample;

subplot(2,1,1)
grid on
hold on
title(titleName1)
xlim([dataX(1) dataX(end)])
xlabel('Time [s]')
ylabel(yName1)
plot(dataX,rho0,'-g',dataX,rho,'-b',dataX,deltaRho,'-r')
legend('rho0','rho','d*rho')
 
subplot(2,1,2)
grid on
hold on
title(titleName2)
plot(dataX,dataY,'-k')
legend(ratioName)
xlim([dataX(1) dataX(end)])
xlabel('Time [s]')
ylabel(yName2)


hold off
end

