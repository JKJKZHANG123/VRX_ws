%% Plot report3 static-route and M5 multi-goal diagnostics.
% Run from the repository root. No optional toolboxes are required.
clear; close all; clc;

root = fileparts(fileparts(mfilename('fullpath')));
staticFile = fullfile(root, 'data', 'static_route_after_bt_fix_20260829_valid.csv');
m5File = fullfile(root, 'data', 'm5_multigoal_20260829_valid.csv');
waypointFile = fullfile(root, 'data', 'm5_multigoal_20260829_valid_waypoints.csv');
S = readtable(staticFile, 'TextType', 'string');
M = readtable(m5File, 'TextType', 'string');
W = readtable(waypointFile, 'TextType', 'string');

figure('Name','Validated navigation routes');
tiledlayout(1,2);
nexttile;
plot(S.pose_x, S.pose_y, 'b-', 'LineWidth', 1.5); hold on;
plot(S.pose_x(1), S.pose_y(1), 'go', 'MarkerFaceColor','g');
plot(S.goal_x(end), S.goal_y(end), 'rx', 'LineWidth', 2, 'MarkerSize', 10);
grid on; axis equal; xlabel('x [m]'); ylabel('y [m]');
title('Static single goal');
legend('trajectory','start','requested goal','Location','best');

nexttile;
plot(M.pose_x, M.pose_y, 'b-', 'LineWidth', 1.5); hold on;
plot(M.pose_x(1), M.pose_y(1), 'go', 'MarkerFaceColor','g');
plot(W.target_x, W.target_y, 'ro--', 'LineWidth', 1.0, ...
    'MarkerFaceColor','r', 'MarkerSize', 6);
for k = 1:height(W)
    text(W.target_x(k)+0.06, W.target_y(k)+0.06, sprintf('WP%d', W.sequence(k)));
end
grid on; axis equal; xlabel('x [m]'); ylabel('y [m]');
title('M5 NavigateThroughPoses');
legend('trajectory','start','requested route','Location','best');

figure('Name','Static-route diagnostics');
tiledlayout(3,1);
nexttile; plot(S.t, S.crosstrack_err, 'LineWidth', 1.2); grid on;
ylabel('cross-track [m]'); title('Static-route tracking error');
nexttile; plot(S.t, S.des_vx, 'LineWidth', 1.2); hold on;
plot(S.t, S.act_vx, 'LineWidth', 1.2); grid on;
ylabel('speed [m/s]'); legend('desired','actual','Location','best');
nexttile; stairs(S.t, S.safety_stop, 'LineWidth', 1.2); grid on;
xlabel('simulation time [s]'); ylabel('safety stop'); ylim([-0.1 1.1]);

figure('Name','M5 diagnostics');
tiledlayout(3,1);
nexttile; plot(M.t, M.path_length, 'LineWidth', 1.2); hold on;
plot(M.t, M.path_min_clearance, 'LineWidth', 1.2); grid on;
ylabel('[m]'); title('M5 path diagnostics');
legend('latest path length','minimum clearance','Location','best');
nexttile; plot(M.t, M.des_vx, 'LineWidth', 1.2); hold on;
plot(M.t, M.act_vx, 'LineWidth', 1.2); grid on;
ylabel('speed [m/s]'); legend('desired','actual','Location','best');
nexttile; plot(M.t, M.dynamic_track_count, 'LineWidth', 1.0); hold on;
plot(M.t, M.dynamic_point_count, 'LineWidth', 1.0); grid on;
xlabel('simulation time [s]'); ylabel('count');
legend('dynamic tracks','predicted points','Location','best');

staticDistance = sum(hypot(diff(S.pose_x), diff(S.pose_y)), 'omitnan');
m5Distance = sum(hypot(diff(M.pose_x), diff(M.pose_y)), 'omitnan');
fprintf('Static: %d samples, %.1f s, status %s\n', ...
    height(S), S.t(end), S.goal_status(end));
fprintf('  final error %.3f m, traveled %.3f m, max cross-track %.3f m\n', ...
    hypot(S.pose_x(end)-S.goal_x(end), S.pose_y(end)-S.goal_y(end)), ...
    staticDistance, max(S.crosstrack_err));
fprintf('M5: %d samples, %.1f s, status %s\n', ...
    height(M), M.t(end), M.goal_status(end));
fprintf('  post-stop final error %.3f m, traveled %.3f m, max cross-track %.3f m\n', ...
    hypot(M.pose_x(end)-W.target_x(end), M.pose_y(end)-W.target_y(end)), ...
    m5Distance, max(M.crosstrack_err));
for k = 1:height(W)
    fprintf('  WP%d passed=%s, elapsed-after-accept=%.3f s, min error=%.3f m\n', ...
        W.sequence(k), W.passed(k), W.elapsed_after_accept_s(k), W.min_error_m(k));
end
