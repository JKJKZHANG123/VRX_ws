function plot_usv_nav_log(csvPath)
%PLOT_USV_NAV_LOG Plot trajectory, control, safety, and cloud-filter metrics.
%   plot_usv_nav_log('data/safety_test_goal_5_0.csv')

if nargin < 1
    csvPath = 'data/nav_log.csv';
end
T = readtable(csvPath, 'VariableNamingRule', 'preserve');
t = T.t;

figure('Name', ['USV navigation: ' csvPath], 'Color', 'w');
tiledlayout(2, 2, 'TileSpacing', 'compact');

nexttile;
plot(T.pose_x, T.pose_y, 'b-', 'LineWidth', 1.5); hold on;
if any(T.goal_active)
    idx = find(T.goal_active, 1, 'last');
    plot(T.goal_x(idx), T.goal_y(idx), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
end
axis equal; grid on; xlabel('x (m)'); ylabel('y (m)');
title('Trajectory'); legend('USV', 'Goal', 'Location', 'best');

nexttile;
yyaxis left;
plot(t, T.des_vx, 'LineWidth', 1.2); hold on;
plot(t, T.des_wz, 'LineWidth', 1.2);
ylabel('Command (m/s, rad/s)');
yyaxis right;
plot(t, T.left_thrust, '--', t, T.right_thrust, '--', 'LineWidth', 1.0);
ylabel('Thrust (N)'); grid on; xlabel('t (s)');
title('Command and thrust');
legend('v_x', '\omega_z', 'left', 'right', 'Location', 'best');

nexttile;
plot(t, T.obstacle_pts, 'LineWidth', 1.2); hold on;
if ismember('raw_obstacle_pts', T.Properties.VariableNames)
    plot(t, T.raw_obstacle_pts, 'LineWidth', 1.2);
    plot(t, T.lio_input_pts, 'LineWidth', 1.0);
    plot(t, T.registered_pts, 'LineWidth', 1.0);
    legend('structure', 'raw obstacle', 'LIO input', 'registered', ...
        'Location', 'best');
else
    legend('structure', 'Location', 'best');
end
grid on; xlabel('t (s)'); ylabel('points/frame');
title('Point-cloud density');

nexttile;
plot(t, T.crosstrack_err, 'LineWidth', 1.2); hold on;
plot(t, T.path_min_clearance, 'LineWidth', 1.2);
stairs(t, T.safety_stop, 'r-', 'LineWidth', 1.0);
grid on; xlabel('t (s)'); ylabel('m / state');
title('Tracking and safety');
legend('cross-track error', 'path min clearance', 'safety stop', ...
    'Location', 'best');
end
