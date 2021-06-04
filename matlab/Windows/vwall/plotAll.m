function [rho,dataX] = plotAll(files_dir,pos,delta,deltaB,figName,tSample)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
i_limit = length(pos);
col = 1;

for i=1:i_limit
    

    full_name = [files_dir(pos(i)).folder '\' files_dir(pos(i)).name];
    data=load(full_name);
    % lambda
    xGoal = data(:,1:3);
    x = data(:,4:6);
    xVC = data(:,7:9);
    % beta
    dx_max = data(:,10:12);
    dx = data(:,13:15);
    beta = data(:,6);

    nL = length(beta); % total length

    % Calculating rho, rho0 and rho1 (lambda)
    rho = zeros(1,nL);
    rho1 = rho; rho0 = rho; deltaRho = rho;

    for j=1:length(rho)
       rho0(j) = sqrt((xGoal(j,1)-xVC(j,1))^2 + (xGoal(j,2)-xVC(j,2))^2  + (xGoal(j,3)-xVC(j,3))^2 );
       rho(j) = sqrt((x(j,1)-xVC(j,1))^2 + (x(j,2)-xVC(j,2))^2  + (x(j,3)-xVC(j,3))^2 );

       deltaRho(j) = delta*rho0(j);
    end

    % Calculating rho, rho0 and rho1 (beta)
    rhoB = zeros(1,nL);
    rho1B = rhoB; rho0B = rhoB; deltaRhoB = rhoB;

    for j=1:length(rho)
       rho0B(j) = sqrt(dx_max(j,1)^2 + dx_max(j,2)^2  + dx_max(j,3)^2 );
       rhoB(j) = sqrt((dx(j,1)-dx_max(j,1))^2 + (dx(j,2)-dx_max(j,2))^2  + (dx(j,3)-dx_max(j,3))^2 );

       deltaRhoB(j) = deltaB*rho0B(j);
    end

    dataX = [0:(nL-1)]*tSample;

    if(i==1)
        figure('Name',figName)
    end

    subplot(i_limit,2,col)
    grid on
    hold on
    title([num2str(i) ' - Distance from vc with delta = ' num2str(delta)])
    xlim([dataX(1) dataX(end)])
    xlabel('Time [s]')
    ylabel('Abs pos distance [mm]')
    plot(dataX,rho0,'-g',dataX,rho,'-b',dataX,deltaRho,'-r')
    legend('rho0','rho','d*rho')

    subplot(i_limit,2,col+1)
    grid on
    hold on
    title([num2str(i) '- Velocity diff with deltaB = ' num2str(deltaB)])
    xlim([dataX(1) dataX(end)])
    xlabel('Time [s]')
    ylabel('Abs vel diff [mm/s]')
    plot(dataX,rho0B,'-g',dataX,rhoB,'-b',dataX,deltaRhoB,'-r')
    legend('rho0B','rhoB','dB*rhoB')



    hold off

col = col+2;
end


end

