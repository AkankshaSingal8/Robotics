start = [1, 1];
goal = [20, 20];
obstacles_center = [4.5, 3; 3, 12; 15, 15];
obstacles_radius = [2, 2, 3];

bicycle = bicycleKinematics(VehicleInputs="VehicleSpeedHeadingRate",MaxSteeringAngle=pi/8);

table = readtable('path.csv');
%disp(table)
waypoints = table2array(table);
              
time = 0:0.05:20;         
initial = [waypoints(1,:)'; pi/2]; 

controller = controllerPurePursuit(Waypoints=waypoints,DesiredLinearVelocity=3,MaxAngularVelocity=3*pi);

goal_points = waypoints(end,:)';
goal_radius = 1;

[tBicycle,bicyclePose] = ode45(@(t,y)derivative(bicycle,y,exampleHelperMobileRobotController(controller,y,goal_points,goal_radius)),time,initial);

bicycle_translations = [bicyclePose(:,1:2) zeros(length(bicyclePose),1)];
bicycle_rot = axang2quat([repmat([0 0 1],length(bicyclePose),1) bicyclePose(:,3)]);

figure
hold on
grid on
axis equal
axis([0 30 0 30])
xlabel('X')
ylabel('Y')
title('Configuration Space')

plot(start(1), start(2), 'b*', 'MarkerSize', 10)
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10)

for i = 1:length(obstacles)
    viscircles(obstacles_center(i, :), obstacles_radius(i),'LineWidth', 1);
end

plot(waypoints(:,1),waypoints(:,2),"kx-",MarkerSize=20);
hold all
plotTransforms(bicycle_translations(1:10:end,:),bicycle_rot(1:10:end,:),MeshFilePath="groundvehicle.stl",MeshColor="b");
hold off