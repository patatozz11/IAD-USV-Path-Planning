clc;
clear;
close all;

% Overlay comparison:
%   IA* + Floyd + DWA
%   IA* + Floyd + LOS + DWA
%
% This script only runs and plots the comparison. It does not change the
% global planner, DWA evaluation function, LOS logic, or kinematic settings.

outDir = fullfile('results', 'fusion_los_overlay_comparison');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

oldFigureVisible = get(0, 'DefaultFigureVisible');
cleanupObj = onCleanup(@() set(0, 'DefaultFigureVisible', oldFigureVisible));

%% 1) Fixed map
gridMap = [
     1 1 0 0 0 1 1 1 0 0 1 1 0 0 0 0 0 0 1 1;
     1 0 0 0 0 0 0 0 0 0 1 0 0 0 1 0 0 0 0 1;
     0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0;
     0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 1 0 0;
     1 1 0 0 0 1 1 0 0 0 1 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 1 0 0 0 1 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 1 1;
     0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 1 1;
     0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     1 1 0 0 0 0 0 0 0 0 1 1 0 0 1 1 0 0 0 1;
     0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 1;
     0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 1 1 1 0 0 0 0 0 0 0 1 0 0 0 0 0;
     0 0 0 0 1 1 1 0 0 0 1 0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 0 1 1 0;
     1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0;
     1 1 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0;
     1 0 0 0 0 1 0 0 0 0 0 0 0 1 1 0 0 0 0 0;
     0 0 0 0 1 1 1 0 0 1 1 0 0 0 0 0 0 0 1 1
];

gridMap = rot90(gridMap, 3);
[MaxRow, MaxCol] = size(gridMap);

start = [1, 1];
goal = [20, 16];

if isObstacleCell(start, gridMap)
    error('The fixed start point [%d, %d] is on an obstacle.', start(1), start(2));
end
if isObstacleCell(goal, gridMap)
    error('The fixed goal point [%d, %d] is on an obstacle.', goal(1), goal(2));
end

moveobs_trajectory = [];
Obs_Unknown = [];

[obs_static_rows, obs_static_cols] = find(gridMap == 1);
obs_Static = [obs_static_rows, obs_static_cols];

%% 2) Shared IA* global path
diagRule = 1;
[Path, distanceX, OPEN_num, OPEN, run_time] = ...
    IAstar_FiveDir_Fallback_Fast(gridMap, start, goal, diagRule); %#ok<ASGLU>

if isempty(Path)
    error('IAstar_FiveDir_Fallback_Fast failed to find a shared global path.');
end

%% 3) Shared DWA and LOS parameters
global dt;
dt = 0.1;

Kinematic = [1.0, toRadian(20.0), 0.2, toRadian(50.0), 0.01, toRadian(1)];
evalParam = [0.05, 0.2, 0.1, 3.0];

d_safe = 0.2;
losConfig.reach_dist = 1.0;
losConfig.sight_dist = 6.0;
losConfig.max_skip = 3;

%% 4) Run final noLOS and LOS fusion functions under identical conditions
set(0, 'DefaultFigureVisible', 'off');

clear DWA_Fusion_noLOS
figure('Visible', 'off');
hold on;
axis equal;
[Final_Path_noLOS, metrics_noLOS] = DWA_Fusion_noLOS( ...
    gridMap, obs_Static, Obs_Unknown, moveobs_trajectory, Path, start, goal, ...
    Kinematic, evalParam);
close(gcf);

clear DWA_Fusion_LOS
figure('Visible', 'off');
hold on;
axis equal;
[Final_Path_LOS, metrics_LOS] = DWA_Fusion_LOS( ...
    gridMap, obs_Static, Obs_Unknown, moveobs_trajectory, Path, start, goal, ...
    Kinematic, evalParam, d_safe, losConfig);
close(gcf);

%% 5) Save metrics
Algorithm = ["IA*+DWA"; "IA*+LOS+DWA"];
Success = [metrics_noLOS.success; metrics_LOS.success];
Path_length_m = [metrics_noLOS.path_length; metrics_LOS.path_length];
Navigation_time_s = [metrics_noLOS.navigation_time; metrics_LOS.navigation_time];
Minimum_obstacle_distance_grid = [ ...
    metrics_noLOS.minimum_obstacle_distance; ...
    metrics_LOS.minimum_obstacle_distance];
Average_obstacle_distance_grid = [ ...
    metrics_noLOS.average_obstacle_distance; ...
    metrics_LOS.average_obstacle_distance];
Cumulative_angular_velocity_change = [ ...
    metrics_noLOS.cumulative_delta_angular_velocity; ...
    metrics_LOS.cumulative_delta_angular_velocity];

T = table(Algorithm, Success, Path_length_m, Navigation_time_s, ...
    Minimum_obstacle_distance_grid, Average_obstacle_distance_grid, ...
    Cumulative_angular_velocity_change);

writetable(T, fullfile(outDir, 'fusion_los_overlay_metrics.csv'));
save(fullfile(outDir, 'fusion_los_overlay_metrics.mat'), ...
    'T', 'gridMap', 'start', 'goal', 'Path', 'Final_Path_noLOS', ...
    'Final_Path_LOS', 'metrics_noLOS', 'metrics_LOS', 'Kinematic', ...
    'evalParam', 'd_safe', 'losConfig', 'diagRule', 'distanceX', ...
    'OPEN_num', 'run_time');

%% 6) Final overlay figure
set(0, 'DefaultFigureVisible', oldFigureVisible);
figOverlay = figure('Color', 'w', 'Name', 'Fusion LOS overlay comparison');
drawFusionOverlay(gca, gridMap, Path, Final_Path_noLOS, Final_Path_LOS, ...
    start, goal, MaxRow, MaxCol);

print(figOverlay, fullfile(outDir, 'fusion_los_overlay_comparison.png'), '-dpng', '-r600');
saveas(figOverlay, fullfile(outDir, 'fusion_los_overlay_comparison.fig'));

figZoom = figure('Color', 'w', 'Name', 'Fusion LOS overlay comparison zoom');
drawFusionOverlay(gca, gridMap, Path, Final_Path_noLOS, Final_Path_LOS, ...
    start, goal, MaxRow, MaxCol);
xlim([10 16]);
ylim([7 13]);
title('Zoomed comparison of IA*+DWA and IA*+LOS+DWA');

print(figZoom, fullfile(outDir, 'fusion_los_overlay_comparison_zoom.png'), '-dpng', '-r600');
saveas(figZoom, fullfile(outDir, 'fusion_los_overlay_comparison_zoom.fig'));

fprintf('\nShared IA* global path:\n');
fprintf('  Length: %.3f | nodes: %d | OPEN nodes: %d | planning time: %.3f ms\n', ...
    distanceX, size(Path, 1), OPEN_num, run_time);
fprintf('\nFusion overlay comparison metrics:\n');
disp(T);
fprintf('\nSaved results to: %s\n', outDir);

function tf = isObstacleCell(p, gridMap)
tf = p(1) < 1 || p(1) > size(gridMap, 1) || ...
     p(2) < 1 || p(2) > size(gridMap, 2) || ...
     gridMap(p(1), p(2)) == 1;
end

function radian = toRadian(degree)
radian = degree / 180 * pi;
end

function drawFusionOverlay(ax, gridMap, Path, Final_Path_noLOS, Final_Path_LOS, start, goal, MaxRow, MaxCol)
axes(ax); %#ok<LAXES>
cla(ax);
hold(ax, 'on');
axis(ax, 'equal');
axis(ax, [1 MaxRow+1, 1 MaxCol+1]);
set(ax, 'XTick', 1:1:MaxRow+1, 'YTick', 1:1:MaxCol+1, ...
    'XGrid', 'on', 'YGrid', 'on', 'GridLineStyle', '-', ...
    'XTickLabelRotation', 0);

for r = 1:MaxRow
    for c = 1:MaxCol
        if gridMap(r, c) == 1
            fill(ax, [r, r+1, r+1, r], [c, c, c+1, c+1], ...
                'k', 'EdgeColor', 'none', 'HandleVisibility', 'off');
        end
    end
end

hGlobal = plot(ax, Path(:,1)+0.5, Path(:,2)+0.5, ...
    'k:', 'LineWidth', 1.4);
hLOS = plot(ax, Final_Path_LOS(:,1)+0.5, Final_Path_LOS(:,2)+0.5, ...
    'b-', 'LineWidth', 2.2);
hNoLOS = plot(ax, Final_Path_noLOS(:,1)+0.5, Final_Path_noLOS(:,2)+0.5, ...
    'r--', 'LineWidth', 2.8);

hStart = plot(ax, start(1)+0.5, start(2)+0.5, ...
    'go', 'MarkerSize', 9, 'LineWidth', 1.8, 'MarkerFaceColor', 'g');
set(hStart, 'HandleVisibility', 'off');
hGoal = plot(ax, goal(1)+0.5, goal(2)+0.5, ...
    'ro', 'MarkerSize', 9, 'LineWidth', 1.8, 'MarkerFaceColor', 'r');
set(hGoal, 'HandleVisibility', 'off');

text(ax, start(1)+0.7, start(2)+0.3, 'Start', 'FontSize', 12);
text(ax, goal(1)+0.7, goal(2)+0.5, 'End', 'FontSize', 12);

legendFontSize = 10;
legend(ax, [hGlobal, hNoLOS, hLOS], ...
    {'IA*+Floyd path', 'IA*+Floyd+DWA', 'IA*+Floyd+LOS+DWA'}, ...
    'Location', 'northwest', 'FontSize', legendFontSize);
xlabel(ax, 'X / grid');
ylabel(ax, 'Y / grid');
title(ax, 'Comparison of IA*+DWA and IA*+LOS+DWA');
end
