function plot_ekf_ab_comparison()
% Compare Point-LIO and a passive robot_localization EKF in VRX.
% Run from any directory; paths are resolved relative to this file.

here = fileparts(mfilename('fullpath'));
dataDir = fullfile(here, '..', 'data');
staticBefore = readtable(fullfile(dataDir, 'ekf_ab_static_before.csv'));
staticAfter = readtable(fullfile(dataDir, 'ekf_ab_static_after.csv'));
runningBefore = readtable(fullfile(dataDir, 'ekf_ab_running_before.csv'));
runningAfter = readtable(fullfile(dataDir, 'ekf_ab_running_after.csv'));
summary = readtable(fullfile(dataDir, 'ekf_ab_metrics_summary.csv'));

% 1. Static error and drift. The EKF curve exists only in *_after.
figure('Name', 'EKF A/B - Static', 'Color', 'w');
tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
nexttile;
plot(staticBefore.elapsed_s, staticBefore.lio_truth_xy_error_m, 'r-', 'LineWidth', 1.2); hold on;
plot(staticAfter.elapsed_s, staticAfter.lio_truth_xy_error_m, 'b-', 'LineWidth', 1.2);
plot(staticAfter.elapsed_s, staticAfter.ekf_truth_xy_error_m, 'g-', 'LineWidth', 1.2);
grid on; xlabel('时间 / s'); ylabel('平面位置误差 / m');
title('静止：真值位置误差');
legend('无 EKF：LIO', '有 EKF：LIO', '有 EKF：EKF', 'Location', 'best');
nexttile;
plot(staticBefore.elapsed_s, staticBefore.lio_xy_drift_m, 'r-', 'LineWidth', 1.2); hold on;
plot(staticAfter.elapsed_s, staticAfter.lio_xy_drift_m, 'b-', 'LineWidth', 1.2);
plot(staticAfter.elapsed_s, staticAfter.ekf_xy_drift_m, 'g-', 'LineWidth', 1.2);
grid on; xlabel('时间 / s'); ylabel('相对起点漂移 / m');
title('静止：平面漂移');
legend('无 EKF：LIO', '有 EKF：LIO', '有 EKF：EKF', 'Location', 'best');

% 2. Running trajectories. All three tracks are in camera_init-relative axes.
figure('Name', 'EKF A/B - Running Trajectory', 'Color', 'w');
plot(runningBefore.truth_x_rel_m, runningBefore.truth_y_rel_m, 'k--', 'LineWidth', 1.4); hold on;
plot(runningBefore.lio_x_rel_m, runningBefore.lio_y_rel_m, 'r-', 'LineWidth', 1.1);
plot(runningAfter.truth_x_rel_m, runningAfter.truth_y_rel_m, 'c--', 'LineWidth', 1.4);
plot(runningAfter.lio_x_rel_m, runningAfter.lio_y_rel_m, 'b-', 'LineWidth', 1.1);
plot(runningAfter.ekf_camera_x_rel_m, runningAfter.ekf_camera_y_rel_m, 'g-', 'LineWidth', 1.2);
grid on; axis equal; xlabel('相对 X / m'); ylabel('相对 Y / m');
title('运行：真值、Point-LIO 与 EKF 航迹');
legend('无 EKF：真值', '无 EKF：LIO', '有 EKF：真值', ...
    '有 EKF：LIO', '有 EKF：EKF', 'Location', 'best');

% 3. Running tracking error and safety status.
figure('Name', 'EKF A/B - Running Error', 'Color', 'w');
tiledlayout(2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
nexttile;
plot(runningBefore.elapsed_s, runningBefore.lio_truth_xy_error_m, 'r-', 'LineWidth', 1.2); hold on;
plot(runningAfter.elapsed_s, runningAfter.lio_truth_xy_error_m, 'b-', 'LineWidth', 1.2);
plot(runningAfter.elapsed_s, runningAfter.ekf_truth_xy_error_m, 'g-', 'LineWidth', 1.2);
grid on; xlabel('时间 / s'); ylabel('平面位置误差 / m');
title('运行：位置跟踪误差');
legend('无 EKF：LIO', '有 EKF：LIO', '有 EKF：EKF', 'Location', 'best');
nexttile;
plot(runningBefore.elapsed_s, runningBefore.safety_stop, 'r-'); hold on;
plot(runningAfter.elapsed_s, runningAfter.safety_stop, 'g-');
grid on; ylim([-0.05, 1.05]); xlabel('时间 / s'); ylabel('Safety stop');
title('运行：安全停止状态（应不持续为 1）');
legend('无 EKF', '有 EKF', 'Location', 'best');

% 4. Summary bar chart. Rows are ordered: static before/after, running before/after.
labels = categorical({'静止 LIO 无EKF', '静止 LIO 有EKF', '静止 EKF', ...
    '运行 LIO 无EKF', '运行 LIO 有EKF', '运行 EKF'});
labels = reordercats(labels, cellstr(labels));
rmse = [summary.lio_truth_xy_error_m_rmse(1), ...
    summary.lio_truth_xy_error_m_rmse(2), summary.ekf_truth_xy_error_m_rmse(2), ...
    summary.lio_truth_xy_error_m_rmse(3), summary.lio_truth_xy_error_m_rmse(4), ...
    summary.ekf_truth_xy_error_m_rmse(4)];
p95 = [summary.lio_truth_xy_error_m_p95(1), ...
    summary.lio_truth_xy_error_m_p95(2), summary.ekf_truth_xy_error_m_p95(2), ...
    summary.lio_truth_xy_error_m_p95(3), summary.lio_truth_xy_error_m_p95(4), ...
    summary.ekf_truth_xy_error_m_p95(4)];
figure('Name', 'EKF A/B - Metrics', 'Color', 'w');
bar(labels, [rmse(:), p95(:)]);
grid on; ylabel('误差 / m'); title('RMSE 与 P95 平面位置误差（越低越好）');
legend('RMSE', 'P95', 'Location', 'northwest');

fprintf('\nEKF A/B 数据已加载：\n');
fprintf('静止有 EKF：LIO RMSE %.4f m，EKF RMSE %.4f m\n', ...
    summary.lio_truth_xy_error_m_rmse(2), summary.ekf_truth_xy_error_m_rmse(2));
fprintf('运行有 EKF：LIO RMSE %.4f m，EKF RMSE %.4f m\n', ...
    summary.lio_truth_xy_error_m_rmse(4), summary.ekf_truth_xy_error_m_rmse(4));
fprintf(['注意：无 EKF 与有 EKF 是独立冷启动试验；运行时两次实际航程不同，' ...
    '严格算法对比应优先看有 EKF 文件中同时记录的 LIO/EKF 列。\n']);
end
