function PlotFollow(robot, Waypoints, Path, JointPath,limit)
%Small animation to show how the robot follows a trajectory.
%[result] = PlotFollow(robot, Waypoints, Path, JointPath, limit)
%- robot: input rigidBodyTree.
%- Waypoints, Path: trajectory and middle points in cartesian space.
%- JointPath: trajectory in joint space.
%- limit: finishes the animation n interactions earlier.

figure
show(robot, JointPath(1,:)','Frames','off','PreservePlot',false);
hold all
plot3(Path(:,1),Path(:,2),Path(:,3),'-r','LineWidth', 2);
plot3(Waypoints(:,1),Waypoints(:,2),Waypoints(:,3),'o','LineWidth', 2);

title('Trajectory execution')
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
axis([-0.6 0.6 -0.6 0.6 -0.1 0.8])

size_JP = size(JointPath);
limit_t = size_JP(1);
if (limit>0)
    if (limit<size_JP(1)+1)
    limit_t = size_JP(1)-limit;
    end
end

for i = 1:limit_t
    show(robot, JointPath(i,:)','Frames','off','PreservePlot',false);
    pause(0.5)
    drawnow
end
end