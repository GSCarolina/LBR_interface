function PlotTrajectory(Waypoints, Path, JointPath)
%Plots the current trajecotry.
%[result] = PlotTrajectory(Waypoints, Path, JointPath)
%- Waypoints: input nx3 vector with the desired points.
%- Path: input n*Nx3 vector complete trajectory in cartesian space.
%- Joint Path: input with trajectory in joint space.
%- result: output message.

%Cartesian space
figure
hold all
grid on
plot3(Path(:,1),Path(:,2),Path(:,3),'-r','LineWidth', 2);
plot3(Waypoints(:,1),Waypoints(:,2),Waypoints(:,3),'o','LineWidth', 2);
title('Trajectory in Cartesian space (3D)')
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
axis([-0.6 0.6 -0.6 0.6 -0.6 0.6])
hold off

%Joint-space
figure
maximun = zeros(1,6);
minimum = zeros(1,6);
for i=1:6
%Matching colors as in SIMULINK scope:
color = '-x';
    switch i
        case 1
           color = '-yx';
        case 2
           color = '-bx';
        case 3
           color = '-rx';
        case 4
           color = '-gx';
        case 5
           color = '-mx';
        case 6
           color = '-cx';
    end

plot(JointPath(:,i)',color);
hold on
maximun(i) = max(JointPath(:,i));
minimum(i) = min(JointPath(:,i));
end
grid on
maxY=max(maximun)+1;
minY=min(minimum)-1;
axis([1 11 minY maxY])
title('Trajectoy in Joint space (2D)')
xlabel('interaction [n]')
ylabel('q [rad]')
legend('q1','q2','q3','q4','q5','q6')
hold off

result = 'Showing 2 new figures';
fprintf('%s\n',result);
end