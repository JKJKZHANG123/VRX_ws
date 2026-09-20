function plot_lio_before_after()
% MATLAB comparison for the VRX Point-LIO stationary A/B test.
% Run this file from any directory; paths are resolved relative to this file.

here = fileparts(mfilename('fullpath'));
dataDir = fullfile(here, '..', 'data');
raw = readtable(fullfile(dataDir, ...
    'raw_lio_stationary_before_drift_fix_20260827.csv'));
improved = readtable(fullfile(dataDir, ...
    'improved_lio_stationary_final_optimized_20260827.csv'));
summary = readtable(fullfile(dataDir, 'lio_metrics_summary.csv'));
pipeline = readtable(fullfile(dataDir, 'pipeline_metrics_summary.csv'));
runtime = readtable(fullfile(dataDir, ...
    'safetycloud_nav2_runtime_validation_20260827.csv'));
candidate = readtable(fullfile(dataDir, ...
    'improved_lio_stationary_after_zupt_candidate_20260827.csv'));

% Each run has an independent world/LIO origin. Align only translations.
rawLioX = raw.lio_x_m - raw.lio_x_m(1);
rawLioY = raw.lio_y_m - raw.lio_y_m(1);
rawTruthX = raw.truth_x_m - raw.truth_x_m(1);
rawTruthY = raw.truth_y_m - raw.truth_y_m(1);
impLioX = improved.lio_x_m - improved.lio_x_m(1);
impLioY = improved.lio_y_m - improved.lio_y_m(1);
impTruthX = improved.truth_x_m - improved.truth_x_m(1);
impTruthY = improved.truth_y_m - improved.truth_y_m(1);

figure('Name', 'Point-LIO Before and After', 'Color', 'w');
tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(rawTruthX, rawTruthY, 'k--', 'LineWidth', 1.3); hold on;
plot(rawLioX, rawLioY, 'r-', 'LineWidth', 1.2);
plot(impTruthX, impTruthY, 'b--', 'LineWidth', 1.3);
plot(impLioX, impLioY, 'g-', 'LineWidth', 1.2);
grid on; axis equal;
xlabel('相对 X / m'); ylabel('相对 Y / m');
title('归一化静止测试轨迹');
legend('基线真值', '基线 LIO', '改进真值', '改进 LIO', ...
    'Location', 'best');

nexttile;
plot(raw.time_s, raw.lio_xy_drift_m, 'r-', 'LineWidth', 1.2); hold on;
plot(improved.time_s, improved.lio_xy_drift_m, 'g-', 'LineWidth', 1.2);
yline(0.3, 'k:', '0.3 m');
grid on; xlabel('时间 / s'); ylabel('LIO 平面漂移 / m');
title('LIO 平面漂移');
legend('原始基线', '改进版本', 'Location', 'best');

nexttile;
plot(raw.time_s, raw.filtered_points, 'r-', 'LineWidth', 1.1); hold on;
plot(raw.time_s, raw.registered_points, 'r--', 'LineWidth', 1.1);
plot(improved.time_s, improved.filtered_points, 'g-', 'LineWidth', 1.1);
plot(improved.time_s, improved.registered_points, 'g--', 'LineWidth', 1.1);
grid on; xlabel('时间 / s'); ylabel('点数');
title('点云链路点数');
legend('基线 filtered', '基线 registered', '改进 filtered', ...
    '改进 registered', 'Location', 'best');

nexttile;
labels = categorical({'末值漂移', '最大漂移', 'P95 漂移', '相对轨迹 RMSE'});
labels = reordercats(labels, {'末值漂移', '最大漂移', 'P95 漂移', '相对轨迹 RMSE'});
rawVals = [summary.final_lio_xy_drift_m(1), summary.max_lio_xy_drift_m(1), ...
    summary.p95_lio_xy_drift_m(1), summary.rmse_relative_xy_error_m(1)];
impVals = [summary.final_lio_xy_drift_m(2), summary.max_lio_xy_drift_m(2), ...
    summary.p95_lio_xy_drift_m(2), summary.rmse_relative_xy_error_m(2)];
bar(labels, [rawVals(:), impVals(:)]);
grid on; ylabel('误差 / m'); title('关键统计量（越低越好）');
legend('原始基线', '改进版本', 'Location', 'northwest');

% Separate figure for the improved runtime pipeline.
figure('Name', 'Improved Pipeline Runtime', 'Color', 'w');
tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

pointNames = {'filtered_points_median', 'registered_points_median', ...
    'structure_points_median', 'raw_obstacle_points_median'};
pointVals = zeros(1, numel(pointNames));
for i = 1:numel(pointNames)
    idx = strcmp(string(pipeline.metric), pointNames{i});
    pointVals(i) = pipeline.value(find(idx, 1));
end
nexttile;
bar(categorical({'filtered', 'registered', 'structure', 'raw obstacle'}), pointVals);
grid on; ylabel('点数'); title('改进链路中位点数');

freqNames = {'filtered_hz_mean', 'registered_hz_mean', 'local_costmap_hz_mean'};
freqVals = zeros(1, numel(freqNames));
for i = 1:numel(freqNames)
    idx = strcmp(string(pipeline.metric), freqNames{i});
    freqVals(i) = pipeline.value(find(idx, 1));
end
nexttile;
bar(categorical({'filtered Hz', 'registered Hz', 'costmap Hz'}), freqVals);
grid on; ylabel('频率 / Hz'); title('改进链路平均频率');

% Compare the newer ZUPT candidate with the current reference.
candidateVals = [candidate.final_drift_m(1), candidate.max_drift_m(1), ...
    candidate.p95_drift_m(1), candidate.rmse_relative_xy_m(1)];
figure('Name', 'Reference versus ZUPT Candidate', 'Color', 'w');
bar(categorical({'末值漂移', '最大漂移', 'P95 漂移', '相对轨迹 RMSE'}), ...
    [impVals(:), candidateVals(:)]);
grid on; ylabel('误差 / m'); title('当前参考与 ZUPT 候选（越低越好）');
legend('当前参考', 'ZUPT 候选', 'Location', 'northwest');

% Separate figure for SafetyCloud/Nav2 startup validation.
figure('Name', 'SafetyCloud and Nav2 Validation', 'Color', 'w');
tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

startupNames = {'nav2_transform_timeout_count', 'cloud_tf_skip_count', ...
    'cloud_error_count'};
startupVals = zeros(1, numel(startupNames));
for i = 1:numel(startupNames)
    idx = strcmp(string(runtime.metric), startupNames{i});
    startupVals(i) = runtime.value(find(idx, 1));
end
nexttile;
bar(categorical({'Nav2 TF wait', 'cloud TF skip', 'cloud ERROR'}), startupVals);
grid on; ylabel('次数'); title('启动日志计数');

cloudNames = {'safety_cloud_points_min', 'safety_cloud_points_median', ...
    'safety_cloud_points_max', 'filtered_cloud_points_median'};
cloudVals = zeros(1, numel(cloudNames));
for i = 1:numel(cloudNames)
    idx = strcmp(string(runtime.metric), cloudNames{i});
    cloudVals(i) = runtime.value(find(idx, 1));
end
nexttile;
bar(categorical({'Safety min', 'Safety median', 'Safety max', ...
    'filtered median'}), cloudVals);
grid on; ylabel('点数'); title('SafetyCloud/过滤点数');

fprintf('\n主对比数据：\n');
fprintf('基线最大漂移: %.3f m\n', rawVals(2));
fprintf('改进最大漂移: %.3f m\n', impVals(2));
fprintf('基线末值漂移: %.3f m\n', rawVals(1));
fprintf('改进末值漂移: %.3f m\n', impVals(1));
fprintf('最大漂移下降: %.1f%%\n', 100 * (rawVals(2) - impVals(2)) / rawVals(2));
fprintf('末值漂移下降: %.1f%%\n', 100 * (rawVals(1) - impVals(1)) / rawVals(1));
fprintf('SafetyCloud 点数范围: %.0f--%.0f，中位数 %.0f\n', ...
    cloudVals(1), cloudVals(3), cloudVals(2));
fprintf('Nav2 Transform 启动等待: %.0f 次（约 2 秒后恢复）\n', startupVals(1));
fprintf(['注意：两次测试不是同一时刻采集；请将结果解释为 A/B 统计比较，' ...
    '不要直接比较绝对 world 坐标。\n']);
end
