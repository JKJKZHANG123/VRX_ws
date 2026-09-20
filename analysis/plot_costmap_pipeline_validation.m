function plot_costmap_pipeline_validation(csvPath)
%PLOT_COSTMAP_PIPELINE_VALIDATION Plot Point-LIO/Nav2 perception health.
%   plot_costmap_pipeline_validation( ...
%     'data/costmap_pipeline_costmap_units_fix_20260827.csv')

if nargin < 1
    csvPath = 'data/costmap_pipeline_costmap_units_fix_20260827.csv';
end
T = readtable(csvPath, 'VariableNamingRule', 'preserve');

figure('Name', 'USV perception pipeline validation', 'Color', 'w');
tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(T.time_s, T.raw_hz, T.time_s, T.filtered_hz, ...
    T.time_s, T.registered_hz, T.time_s, T.structure_hz, ...
    T.time_s, T.raw_obstacle_hz, T.time_s, T.local_costmap_hz, ...
    'LineWidth', 1.1);
grid on; xlabel('wall time (s)'); ylabel('Hz'); title('Perception rates');
legend('raw LiDAR', 'LIO input', 'registered', 'structure', ...
    'raw obstacle', 'local costmap', 'Location', 'best');

nexttile;
yyaxis left;
plot(T.time_s, T.odom_hz, 'LineWidth', 1.1); ylabel('odometry (Hz)');
yyaxis right;
plot(T.time_s, T.clock_hz, 'LineWidth', 1.1); ylabel('/clock (Hz)');
grid on; xlabel('wall time (s)'); title('High-rate timing');
legend('Point-LIO odometry', 'ROS simulation clock', 'Location', 'best');

nexttile;
plot(T.time_s, T.structure_points, T.time_s, T.raw_obstacle_points, ...
    T.time_s, T.safety_points, 'LineWidth', 1.1);
grid on; xlabel('wall time (s)'); ylabel('points/frame');
title('Obstacle-cloud density');
legend('structure', 'raw obstacle', 'safety', 'Location', 'best');

nexttile;
plot(T.time_s, T.costmap_nonfree_cells, T.time_s, ...
    T.costmap_inscribed_cells, T.time_s, T.costmap_lethal_cells, ...
    T.time_s, T.obstacle_layer_lethal_cells, 'LineWidth', 1.1);
grid on; xlabel('wall time (s)'); ylabel('cells');
title('30 m local costmap occupancy');
legend('non-free', 'inscribed + lethal', 'lethal', ...
    'raw obstacle-layer lethal', 'Location', 'best');

nexttile;
valid = isfinite(T.lio_x_m) & isfinite(T.lio_y_m) & ...
    isfinite(T.truth_x_m) & isfinite(T.truth_y_m);
Lx = T.lio_x_m(valid); Ly = T.lio_y_m(valid);
Gx = T.truth_x_m(valid); Gy = T.truth_y_m(valid);
if ~isempty(Lx)
    plot(Lx - Lx(1), Ly - Ly(1), '-o', 'MarkerSize', 3, ...
        'LineWidth', 1.1); hold on;
    plot(Gx - Gx(1), Gy - Gy(1), '-o', 'MarkerSize', 3, ...
        'LineWidth', 1.1);
end
grid on; axis equal; xlabel('\Deltax (m)'); ylabel('\Deltay (m)');
title('Stationary position excursion');
legend('Point-LIO', 'Gazebo truth', 'Location', 'best');

nexttile;
plot(T.time_s, T.safety_age_s, T.time_s, T.costmap_age_s, ...
    'LineWidth', 1.1);
grid on; xlabel('wall time (s)'); ylabel('age (s)');
title('Timestamp freshness');
legend('safety cloud', 'local costmap', 'Location', 'best');

sgtitle(strrep(csvPath, '_', '\_'));
end
