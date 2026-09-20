function summary = plot_lio_stationary_validation(dataDir)
%PLOT_LIO_STATIONARY_VALIDATION Compare stationary Point-LIO configurations.
%   summary = plot_lio_stationary_validation('data')
% Aligns LIO displacement with the Gazebo world using the WAM-V spawn yaw
% (1.0 rad), plots drift/height/point density, and returns a summary table.

if nargin < 1
    dataDir = 'data';
end
cases = {
    'Before drift fix (z >= -1.5 m)', 'lio_stationary_before_drift_fix_20260827.csv';
    'Initial z-filter result (z >= -0.5 m)', 'lio_stationary_after_drift_fix_20260827.csv';
    'Final optimized (z >= -0.5 m, filter=2)', 'lio_stationary_final_optimized_20260827.csv';
    'Candidate: lidar covariance 0.05', 'lio_stationary_tune_cov005_20260827.csv';
    'Candidate: z >= -0.8 m', 'lio_stationary_tune_z08_20260827.csv';
    'Candidate: point filter=3', 'lio_stationary_tune_filter3_20260827.csv';
    'Fake plane + LiDAR only', 'lio_stationary_water_filter_20260827.csv';
    'Plane hidden + LiDAR only', 'lio_stationary_visibility_fix_20260827.csv';
    'Plane hidden + IMU + z filter', 'lio_stationary_visibility_fix_imu_input_20260827.csv';
    'Plane hidden + IMU + all finite', 'lio_stationary_visibility_fix_imu_finite_only_20260827.csv';
    'Final z-filter smoke test', 'lio_stationary_final_smoke_20260827.csv'
};
spawnYaw = 1.0;
R = [cos(spawnYaw), -sin(spawnYaw); sin(spawnYaw), cos(spawnYaw)];
colors = lines(size(cases, 1));
records = cell(size(cases, 1), 1);
valid = false(size(cases, 1), 1);
metrics = nan(size(cases, 1), 6);

for k = 1:size(cases, 1)
    path = fullfile(dataDir, cases{k, 2});
    if ~isfile(path)
        warning('Missing validation file: %s', path);
        continue;
    end
    T = readtable(path, 'VariableNamingRule', 'preserve');
    truth = [T.truth_x_m - T.truth_x_m(1), T.truth_y_m - T.truth_y_m(1)];
    lio = [T.lio_x_m - T.lio_x_m(1), T.lio_y_m - T.lio_y_m(1)];
    lioWorld = (R * lio')';
    xyError = vecnorm(lioWorld - truth, 2, 2);
    zDrift = T.lio_z_m - T.lio_z_m(1);
    points = T.filtered_points(T.filtered_points > 0);
    records{k} = struct('T', T, 'truth', truth, 'lio', lioWorld, ...
        'xyError', xyError, 'zDrift', zDrift);
    valid(k) = true;
    metrics(k, :) = [xyError(end), max(xyError), ...
        sqrt(mean(xyError.^2)), min(zDrift), max(zDrift), median(points)];
end

figure('Name', 'Point-LIO stationary validation', 'Color', 'w');
tiledlayout(2, 2, 'TileSpacing', 'compact');

nexttile; hold on;
for k = find(valid)'
    D = records{k};
    plot(D.lio(:,1), D.lio(:,2), 'Color', colors(k,:), 'LineWidth', 1.3);
end
k0 = find(valid, 1, 'last');
if ~isempty(k0)
    plot(records{k0}.truth(:,1), records{k0}.truth(:,2), 'k--', 'LineWidth', 1.8);
end
axis equal; grid on; xlabel('world \Deltax (m)'); ylabel('world \Deltay (m)');
title('Start-relative trajectory');
legend([cases(valid,1); {'Gazebo truth'}], 'Location', 'best');

nexttile; hold on;
for k = find(valid)'
    plot(records{k}.T.time_s, records{k}.xyError, ...
        'Color', colors(k,:), 'LineWidth', 1.2);
end
grid on; xlabel('time (s)'); ylabel('aligned XY error (m)');
title('LIO-to-truth displacement error');

nexttile; hold on;
for k = find(valid)'
    plot(records{k}.T.time_s, records{k}.zDrift, ...
        'Color', colors(k,:), 'LineWidth', 1.2);
end
grid on; xlabel('time (s)'); ylabel('\Deltaz (m)');
title('Vertical drift');

nexttile; hold on;
for k = find(valid)'
    plot(records{k}.T.time_s, records{k}.T.filtered_points, ...
        'Color', colors(k,:), 'LineWidth', 1.1);
end
grid on; xlabel('time (s)'); ylabel('points/frame');
title('Point-LIO input density');
legend(cases(valid,1), 'Location', 'best');

summary = array2table(metrics(valid,:), 'VariableNames', ...
    {'FinalXYError_m','MaxXYError_m','RMSEXY_m','MinZDrift_m', ...
     'MaxZDrift_m','MedianFilteredPoints'});
summary.Configuration = cases(valid,1);
summary = movevars(summary, 'Configuration', 'Before', 1);
disp(summary);
end
