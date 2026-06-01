clc; clear; close all;

%% ── CONSTANTS ────────────────────────────────────────
g    = 9.81;        % gravity (m/s²)
v0   = 12.5;        % measured muzzle velocity (m/s)
h    = 1.016;       % barrel height above floor (m) - 40 inches
m    = 0.0006;      % dart mass ~0.6g (kg)
d    = 0.0125;      % dart diameter 12.5mm (m)
Cd   = 0.45;        % drag coefficient (cylinder)
rho  = 1.225;       % air density (kg/m³)
A    = pi*(d/2)^2;  % frontal area (m²)
k    = 0.5*Cd*rho*A; % drag constant

%% ── MEASURED MUZZLE VELOCITY DATA ───────────────────
% Drop test measurements (40 inch barrel height)
% distance = v0 * sqrt(2h/g)

% Time of flight at 40 inches height
t_drop = sqrt(2*h/g);
fprintf('Time of flight (horizontal): %.3f seconds\n\n', t_drop)

% Your 5 measured shot distances (meters)
distances = [5.359, 6.223, 6.477, 5.131, 5.994];

% Calculate muzzle velocity for each shot
v0_measured = distances / t_drop;

% Statistics
fprintf('=== MUZZLE VELOCITY CHARACTERIZATION ===\n')
fprintf('Shot 1: %.2f m/s\n', v0_measured(1))
fprintf('Shot 2: %.2f m/s\n', v0_measured(2))
fprintf('Shot 3: %.2f m/s\n', v0_measured(3))
fprintf('Shot 4: %.2f m/s\n', v0_measured(4))
fprintf('Shot 5: %.2f m/s\n', v0_measured(5))
fprintf('\n')
fprintf('Mean:         %.2f m/s\n', mean(v0_measured))
fprintf('Std dev:      %.3f m/s\n', std(v0_measured))
fprintf('Max:          %.2f m/s\n', max(v0_measured))
fprintf('Min:          %.2f m/s\n', min(v0_measured))
fprintf('Consistency:  %.1f%%\n\n', (1-std(v0_measured)/mean(v0_measured))*100)

% 95% confidence interval
ci = 1.96 * std(v0_measured) / sqrt(length(v0_measured));
fprintf('95%% CI: %.2f ± %.3f m/s\n\n', mean(v0_measured), ci)

%% ── FIGURE 1: SHOT BY SHOT VELOCITY ─────────────────
figure(1)
bar(v0_measured, 'FaceColor', [0.2 0.6 1.0])
hold on
yline(mean(v0_measured), 'r--', 'LineWidth', 2, ...
    'Label', sprintf('Mean = %.2f m/s', mean(v0_measured)))
yline(mean(v0_measured)+std(v0_measured), 'g--', 'LineWidth', 1, ...
    'Label', '+1σ')
yline(mean(v0_measured)-std(v0_measured), 'g--', 'LineWidth', 1, ...
    'Label', '-1σ')
xlabel('Shot Number')
ylabel('Muzzle Velocity (m/s)')
title('Dart Muzzle Velocity — Shot by Shot')
grid on
ylim([0 max(v0_measured)*1.2])

%% ── FIGURE 2: VELOCITY DISTRIBUTION ─────────────────
figure(2)
histogram(v0_measured, 5, 'Normalization', 'pdf', ...
    'FaceColor', [0.2 0.6 1.0], 'FaceAlpha', 0.7)
hold on
x = linspace(min(v0_measured)-2, max(v0_measured)+2, 100);
y = normpdf(x, mean(v0_measured), std(v0_measured));
plot(x, y, 'r-', 'LineWidth', 2)
xlabel('Muzzle Velocity (m/s)')
ylabel('Probability Density')
title('Muzzle Velocity Distribution')
legend('Measured shots', 'Normal distribution fit')
grid on

%% ── TRAJECTORY FUNCTIONS ─────────────────────────────

function [x, y] = simple_trajectory(v0, theta_deg, h, g)
    % No drag trajectory
    theta = deg2rad(theta_deg);
    vx = v0 * cos(theta);
    vy = v0 * sin(theta);
    a = -0.5*g;
    b = vy;
    c = h;
    disc = b^2 - 4*a*c;
    t_flight = max((-b+sqrt(disc))/(2*a), (-b-sqrt(disc))/(2*a));
    t = linspace(0, t_flight, 1000);
    x = vx * t;
    y = h + vy*t - 0.5*g*t.^2;
end

function [x, y] = drag_trajectory(v0, theta_deg, h, g, k, m)
    % With aerodynamic drag
    theta = deg2rad(theta_deg);
    vx = v0 * cos(theta);
    vy = v0 * sin(theta);
    dt = 0.001;
    x = 0; y = h;
    x_arr = x; y_arr = y;
    while y >= 0
        v   = sqrt(vx^2 + vy^2);
        Fd  = k * v^2;
        ax  = -(Fd/m) * (vx/v);
        ay  = -g - (Fd/m) * (vy/v);
        vx  = vx + ax*dt;
        vy  = vy + ay*dt;
        x   = x  + vx*dt;
        y   = y  + vy*dt;
        x_arr(end+1) = x;
        y_arr(end+1) = y;
    end
    x = x_arr;
    y = y_arr;
end

%% ── FIGURE 3: TRAJECTORY CURVES ─────────────────────
figure(3)
angles  = [0, 5, 10, 15, 20, 30, 45];
colors  = lines(length(angles));
legends = {};

for i = 1:length(angles)
    [x1, y1] = simple_trajectory(v0, angles(i), h, g);
    [x2, y2] = drag_trajectory(v0, angles(i), h, g, k, m);
    plot(x1, y1, '--', 'Color', colors(i,:), 'LineWidth', 1.5)
    hold on
    plot(x2, y2, '-',  'Color', colors(i,:), 'LineWidth', 2)
    legends{end+1} = sprintf('%d° no drag', angles(i));
    legends{end+1} = sprintf('%d° with drag', angles(i));
end

yline(0, 'k-', 'LineWidth', 1)
xlabel('Horizontal Distance (m)')
ylabel('Height (m)')
title(sprintf('Dart Trajectory at Different Angles (v0 = %.1f m/s)', v0))
legend(legends, 'Location', 'best', 'FontSize', 7)
grid on
ylim([-0.1 inf])

%% ── FIGURE 4: RANGE VS LAUNCH ANGLE ─────────────────
figure(4)
angles_full   = -10:0.5:60;
ranges_simple = zeros(size(angles_full));
ranges_drag   = zeros(size(angles_full));

for i = 1:length(angles_full)
    [x1, ~] = simple_trajectory(v0, angles_full(i), h, g);
    [x2, ~] = drag_trajectory(v0, angles_full(i), h, g, k, m);
    ranges_simple(i) = max(x1);
    ranges_drag(i)   = max(x2);
end

plot(angles_full, ranges_simple, 'b--', 'LineWidth', 2)
hold on
plot(angles_full, ranges_drag, 'r-', 'LineWidth', 2)

[max_r1, idx1] = max(ranges_simple);
[max_r2, idx2] = max(ranges_drag);
plot(angles_full(idx1), max_r1, 'b*', 'MarkerSize', 12)
plot(angles_full(idx2), max_r2, 'r*', 'MarkerSize', 12)

xlabel('Launch Angle (degrees)')
ylabel('Range (m)')
title('Dart Range vs Launch Angle')
legend('No drag', 'With drag', ...
    sprintf('Max no drag: %.1fm at %.0f°', max_r1, angles_full(idx1)), ...
    sprintf('Max with drag: %.1fm at %.0f°', max_r2, angles_full(idx2)))
grid on

fprintf('=== RANGE ANALYSIS ===\n')
fprintf('Max range (no drag): %.2f m at %.1f°\n', max_r1, angles_full(idx1))
fprintf('Max range (drag):    %.2f m at %.1f°\n\n', max_r2, angles_full(idx2))

%% ── FIGURE 5: REQUIRED AIM ANGLE PER DISTANCE ───────
figure(5)
target_distances = 0.5:0.1:5.0;
aim_angles = zeros(size(target_distances));

for i = 1:length(target_distances)
    d_target   = target_distances(i);
    angle_low  = -10;
    angle_high = 45;
    for iter = 1:100
        angle_mid = (angle_low + angle_high) / 2;
        [x, ~]    = drag_trajectory(v0, angle_mid, h, g, k, m);
        if max(x) < d_target
            angle_low  = angle_mid;
        else
            angle_high = angle_mid;
        end
    end
    aim_angles(i) = angle_mid;
end

plot(target_distances, aim_angles, 'b-', 'LineWidth', 2)
xlabel('Target Distance (m)')
ylabel('Required Launch Angle (degrees)')
title('Required Aim Angle vs Target Distance')
grid on

% Print lookup table
fprintf('=== AIM ANGLE LOOKUP TABLE ===\n')
fprintf('Distance (m)  |  Launch Angle\n')
fprintf('─────────────────────────────\n')
for d_t = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0]
    idx = find(abs(target_distances - d_t) < 0.05, 1);
    if ~isempty(idx)
        fprintf('    %.1f m      |    %.2f°\n', d_t, aim_angles(idx))
    end
end

%% ── FIGURE 6: TRAJECTORY WITH UNCERTAINTY ───────────
figure(6)
rng(42)
v0_mean = mean(v0_measured);
v0_std  = std(v0_measured);

for i = 1:50
    v0_rand = v0_mean + v0_std * randn();
    [x, y]  = drag_trajectory(v0_rand, 0, h, g, k, m);
    plot(x, y, 'b-', 'LineWidth', 0.5, 'Color', [0.2 0.6 1.0 0.15])
    hold on
end

[x_mean, y_mean] = drag_trajectory(v0_mean, 0, h, g, k, m);
plot(x_mean, y_mean, 'r-', 'LineWidth', 3)
yline(0, 'k-')
xlabel('Horizontal Distance (m)')
ylabel('Height (m)')
title(sprintf('Trajectory Uncertainty — 50 Simulations (σ = %.3f m/s)', v0_std))
legend('Individual shots', 'Mean trajectory')
grid on

%% ── FIGURE 7: PWM VS MUZZLE VELOCITY ────────────────
% Fill in after testing different PWM values
% Format: [pwm_value, measured_distance_meters]

pwm_values  = [150,  175,  200,  225,  255];
% Replace zeros with your measured distances at each PWM
distances_pwm = [0,    0,    0,    0,    5.36]; % shot 1 at PWM 255

% Only plot values you have measured
measured_idx = distances_pwm > 0;
if sum(measured_idx) > 1
    v0_pwm = distances_pwm(measured_idx) / t_drop;
    pwm_measured = pwm_values(measured_idx);

    p = polyfit(pwm_measured, v0_pwm, 1);
    pwm_fit = linspace(min(pwm_measured), max(pwm_measured), 100);
    v0_fit  = polyval(p, pwm_fit);

    figure(7)
    plot(pwm_measured, v0_pwm, 'bo', 'MarkerSize', 8, 'LineWidth', 2)
    hold on
    plot(pwm_fit, v0_fit, 'r-', 'LineWidth', 2)
    xlabel('PWM Value (0-255)')
    ylabel('Muzzle Velocity (m/s)')
    title('Motor PWM vs Dart Muzzle Velocity')
    legend('Measured', 'Linear fit')
    grid on

    fprintf('\n=== PWM CHARACTERIZATION ===\n')
    fprintf('v0 = %.4f × PWM + %.4f\n', p(1), p(2))
end

%% ── SUMMARY ──────────────────────────────────────────
fprintf('\n=== FULL SYSTEM SUMMARY ===\n')
fprintf('Muzzle velocity:    %.2f m/s (%.1f mph)\n', v0, v0*2.237)
fprintf('Barrel height:      %.3f m (%.1f inches)\n', h, h/0.0254)
fprintf('Dart mass:          %.1f g\n', m*1000)
fprintf('Drag coefficient:   %.2f\n', Cd)
fprintf('\nPredicted ranges (with drag):\n')
for ang = [0, 5, 10, 15, 20, 30, 45]
    [x, ~] = drag_trajectory(v0, ang, h, g, k, m);
    fprintf('  %2d°: %.2f m (%.1f ft)\n', ang, max(x), max(x)*3.281)
end
