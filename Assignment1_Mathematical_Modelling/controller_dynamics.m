l = 1;
m1 = 1; % mass
m2 = 1;
g = 10; % gravity
a1 = 1; % link length
a2 = 1;
k1 = 300;
k2 = 10;

q0 = [0; 0];
qdot0 = [0; 0];

time = linspace(0, 10, 1000); 

q = zeros(2, length(time));
qdot = zeros(2, length(time));
qd_lst = zeros(2, length(time));
qddot_lst = zeros(2, length(time));

theta1 = zeros(1, length(time));
theta2 = zeros(1, length(time));
tau_val = zeros(2, length(time));



for i = 2:length(time)
    t = time(i);

    qd = [(pi/4) * sin(t); (pi/5) * cos(t)];
    qd_lst(:, i) = qd;
    qdt = [(pi/4) * cos(t); (- (pi/5) * sin(t))];
    qddot_lst(:, i) = qdt;
    
    tau = (-k1 * (q(:, i-1) - qd)) - (k2 * (qdot(:, i-1) - qdt));
    tau_val(:, i) = tau;
    
    [t_next, y] = ode45(@(t, y) ode_solver(t, y, tau), [time(i-1), time(i)], [q(:, i-1); qdot(:, i-1)]);
    
    
    q(:, i) = y(end, 1:2)';
    qdot(:, i) = y(end, 3:4)';
    theta1(i) = q(1, i);
    theta2(i) = q(2, i);
end

%graph theta vs ts
figure;
plot(time, q(1, :), 'b', 'LineWidth', 2, 'DisplayName', 'theta1');
title('Theta1 and Theta2 vs time');
hold on;
plot(time, q(2, :), 'r', 'LineWidth', 2, 'DisplayName', 'theta2');
hold off;

e = q - qd_lst; 
edot = qdot - qddot_lst;

%graph e vs ts
figure;
plot(time, e(1, :), 'b', 'LineWidth', 2, 'DisplayName', 'theta1');
title('Error 1 and Error 2 vs time');
hold on;
plot(time, e(2, :), 'r', 'LineWidth', 2, 'DisplayName', 'theta2');
hold off;

% graph edot vs ts
figure;
plot(time, edot(1, :), 'b', 'LineWidth', 2, 'DisplayName', 'E DOT 1');
title('Error dot 1 and Error dot 2 vs time');
hold on;
plot(time, edot(2, :), 'r', 'LineWidth', 2, 'DisplayName', 'E DOT 2');
hold off;

%graph tau vs ts
figure;
plot(time, tau_val(1, :), 'b', 'LineWidth', 2, 'DisplayName', 'E DOT 1');
title('Tau1 and Tau2 vs time');
hold on;
plot(time, tau_val(2, :), 'r', 'LineWidth', 2, 'DisplayName', 'E DOT 2');
hold off;

% Animation
figure;
videofile = VideoWriter('part_2.avi','Uncompressed avi');
open(videofile);

for i = 1:length(theta1)
    theta_x = theta1(i);
    theta2_val = theta2(i);

    x0 = 0;
    y0 = 0;

    y1 = l * sin(theta_x);
    x1 = l * cos(theta_x);

    x2 = l * cos(theta_x) + l * cos(theta_x + theta2_val);
    y2 = l * sin(theta_x) + l * sin(theta_x + theta2_val);


    plot([x0, x1], [y0, y1], [x1, x2], [y1, y2], 'linewidth', 3);
    grid on;
    axis([-2, 2, -2, 2]); 
    pause(0.1); 
    frame = getframe(gcf);
    writeVideo(videofile, frame);
end



close(videofile);