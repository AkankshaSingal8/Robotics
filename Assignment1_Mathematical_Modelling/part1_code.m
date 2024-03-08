l = 1;
m1 = 1; % mass
m2 = 1;
g = 10; % gravity
a1 = 1; % link length
a2 = 1;
k1 = 100;
k2 = 10;

qd = [pi/3; pi/4];

q = [0; 0];
qdot = [0; 0];

time = linspace(0, 10, 1000);

theta1 = zeros(1, length(time));
theta2 = zeros(1, length(time));

e1 = zeros(1, length(time));
e2 = zeros(1, length(time));

edot1 = zeros(1, length(time));
edot2 = zeros(1, length(time));

tau1 = zeros(1, length(time));
tau2 = zeros(1, length(time));

for t = 2:length(time)
    tau = (-k1 * (q - qd)) - k2 * qdot;
    tau1(t) = tau(1);
    tau2(t) = tau(2);

    V = [(-m2 * a1 * a2 * (2 * qdot(1) * qdot(2) + qdot(2) * qdot(2)) * sin(q(2)));
        m2 * a1 * a2 * q(1) * q(1) * sin(q(2))];

    G = [(m1 + m2) * g * a1 * cos(q(1)) + m2 * g * a2 * cos(q(1) + q(2));
        m2 * g * a2 * cos(q(1) + q(2))];

    M = [(m1 + m2) * a1 * a1 + m2 * a2 * a2 + 2 * m2 * a1 * a2 * cos(q(2)), m2 * a2 * a2 + m2 * a1 * a2 * cos(q(2));
        m2 * a2 * a2 + m2 * a1 * a2 * cos(q(2)),  m2 * a2 * a2];

    qddot = M \ (-V - G + tau);
    
    dt = time(t) - time(t - 1);
    qdot = qdot + qddot * dt;
    q = q + qdot * dt;
    %disp(q);
    
    theta1(t) = q(1);
    theta2(t) = q(2);
    
    e1(t) = q(1) - qd(1);
    e2(t) = q(2) - qd(2);

    edot1(t) = qdot(1);
    edot2(t) = qdot(2);
    
end

%graph theta vs ts
figure;
plot(time, theta1, 'b', 'LineWidth', 2, 'DisplayName', 'theta1');
title('Theta1 and Theta2 vs time');
hold on;
plot(time, theta2, 'r', 'LineWidth', 2, 'DisplayName', 'theta2');
hold off;

%graph e vs ts
figure;
plot(time, e1, 'b', 'LineWidth', 2, 'DisplayName', 'E1');
title('Error 1 and Error 2 vs time');
hold on;
plot(time, e2, 'r', 'LineWidth', 2, 'DisplayName', 'E2');
hold off;

%graph edot vs ts
figure;
plot(time, edot1, 'b', 'LineWidth', 2, 'DisplayName', 'E DOT 1');
title('Error dot 1 and Error dot 2 vs time');
hold on;
plot(time, edot2, 'r', 'LineWidth', 2, 'DisplayName', 'E DOT 2');
hold off;

%graph tau vs ts
figure;
plot(time, tau1, 'b', 'LineWidth', 2, 'DisplayName', 'tau1');
title('Tau1 and Tau2 vs time');
hold on;
plot(time, tau2, 'r', 'LineWidth', 2, 'DisplayName', 'E2');
hold off;

% Animation
figure;
videofile = VideoWriter('part_1.avi','Uncompressed avi');
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