clc;
clear;
close all;

batchMode = true;
outDir = fullfile('results', 'los_ablation_final_no_dynamic');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

oldFigureVisible = get(0, 'DefaultFigureVisible');
if batchMode
    set(0, 'DefaultFigureVisible', 'off');
    diary(fullfile(outDir, 'run_log.txt'));
    diary on;
end

% Final LOS ablation experiment without dynamic obstacles.
%
% Compared methods:
%   1) IA* + Floyd + traditional DWA
%   2) IA* + Floyd + LOS + traditional DWA
%
% This script does not implement a simplified DWA or LOS internally.
% It only prepares the shared experiment conditions, calls the final
% root-level fusion functions, and summarizes the returned metrics.

%% 1) Function path check
disp('Function path check:');
disp(['DWA_Fusion_noLOS: ', which('DWA_Fusion_noLOS')]);
disp(['DWA_Fusion_LOS  : ', which('DWA_Fusion_LOS')]);
disp(['DynamicWindowApproach: ', which('DynamicWindowApproach')]);
disp(['Evaluation: ', which('Evaluation')]);
disp(['IAstar_FiveDir_Fallback_Fast: ', which('IAstar_FiveDir_Fallback_Fast')]);
disp(['Floyd_Smooth_safer: ', which('Floyd_Smooth_safer')]);

%% 2) Shared map and fixed experiment settings
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

% Fixed points for reproducible ablation. Edit these if your manual run
% used different points.
start = [1, 1];
goal  = [20, 16];

% No moving obstacle is used in this reproducible ablation.
moveObs_Start = [];
moveObs_Goal  = [];
moveobs_trajectory = [];

% No unknown static obstacle is used in this reproducible ablation.
Obs_Unknown = [];

diagRule = 1;
d_safe = 0.2;

global dt;
dt = 0.1;
Kinematic = [1.0, toRadian(20.0), 0.2, toRadian(50.0), 0.01, toRadian(1)];
evalParam = [0.05, 0.2, 0.1, 3.0];

%% 3) Shared IA* + Floyd global path
[Path, distanceX, OPEN_num, ~, run_time] = ...
    IAstar_FiveDir_Fallback_Fast(gridMap, start, goal, diagRule);

if isempty(Path)
    error('IA* failed to generate the shared global path for LOS ablation.');
end

[obs_static_rows, obs_static_cols] = find(gridMap == 1);
obs_Static = [obs_static_rows, obs_static_cols];

%% 4) Run final noLOS and LOS fusion functions under identical conditions
clear DWA_Fusion_noLOS
fprintf('\n\n===== Running IA* + Floyd + DWA =====\n');
prepareBatchFigure(batchMode);
[Final_Path_noLOS, metrics_noLOS] = DWA_Fusion_noLOS( ...
    gridMap, obs_Static, Obs_Unknown, moveobs_trajectory, Path, start, goal, ...
    Kinematic, evalParam);

clear DWA_Fusion_LOS
fprintf('\n\n===== Running IA* + Floyd + LOS + DWA =====\n');
prepareBatchFigure(batchMode);
[Final_Path_LOS, metrics_LOS] = DWA_Fusion_LOS( ...
    gridMap, obs_Static, Obs_Unknown, moveobs_trajectory, Path, start, goal, ...
    Kinematic, evalParam, d_safe);

%% 5) Build and save final comparison table
Algorithm = ["IA*+Floyd+DWA"; "IA*+Floyd+LOS+DWA"];
Success = [metrics_noLOS.success; metrics_LOS.success];
Path_length = [metrics_noLOS.path_length; metrics_LOS.path_length];
Navigation_time = [metrics_noLOS.navigation_time; metrics_LOS.navigation_time];
Minimum_obstacle_distance = [metrics_noLOS.minimum_obstacle_distance; metrics_LOS.minimum_obstacle_distance];
Average_obstacle_distance = [metrics_noLOS.average_obstacle_distance; metrics_LOS.average_obstacle_distance];
Cumulative_angular_velocity_change = [ ...
    metrics_noLOS.cumulative_delta_angular_velocity; ...
    metrics_LOS.cumulative_delta_angular_velocity];

T = table(Algorithm, Success, Path_length, Navigation_time, ...
    Minimum_obstacle_distance, Average_obstacle_distance, ...
    Cumulative_angular_velocity_change);

fprintf('\nShared IA* + Floyd path:\n');
fprintf('  Path length: %.3f | nodes: %d | OPEN nodes: %d | planning time: %.3f ms\n', ...
    distanceX, size(Path, 1), OPEN_num, run_time);
fprintf('\nFinal LOS ablation metrics:\n');
disp(T);

writetable(T, fullfile(outDir, 'los_ablation_no_dynamic_metrics.csv'));
save(fullfile(outDir, 'los_ablation_no_dynamic_metrics.mat'), ...
    'T', 'metrics_noLOS', 'metrics_LOS', 'Path', 'Final_Path_noLOS', ...
    'Final_Path_LOS', 'gridMap', 'start', 'goal', 'moveObs_Start', ...
    'moveObs_Goal', 'moveobs_trajectory', 'Obs_Unknown', 'Kinematic', ...
    'evalParam', 'd_safe', 'diagRule', 'distanceX', 'OPEN_num', 'run_time');

%% 6) Save trajectory comparison figure
plotFinalTrajectoryComparison(gridMap, Path, Final_Path_noLOS, Final_Path_LOS, ...
    start, goal, Obs_Unknown, moveobs_trajectory);
saveas(gcf, fullfile(outDir, 'los_ablation_no_dynamic_trajectory_comparison.fig'));
print(gcf, fullfile(outDir, 'los_ablation_no_dynamic_trajectory_comparison.png'), '-dpng', '-r600');

fprintf('\nSaved results to: %s\n', outDir);

if batchMode
    diary off;
    set(0, 'DefaultFigureVisible', oldFigureVisible);
end

%% Helper functions
function prepareBatchFigure(batchMode)
    if batchMode
        close all;
        figure('Visible', 'off', 'Color', 'w');
        hold on;
        axis equal;
    end
end

function plotFinalTrajectoryComparison(gridMap, globalPath, pathNoLOS, pathLOS, start, goal, obsUnknown, moveobsTrajectory)
    [maxRow, maxCol] = size(gridMap);
    figure('Color', 'w');
    hold on;
    axis equal;
    axis([1 maxRow+1, 1 maxCol+1]);
    set(gca, 'XTick', 1:maxRow+1, 'YTick', 1:maxCol+1, ...
        'XGrid', 'on', 'YGrid', 'on', 'GridLineStyle', '-', ...
        'XTickLabelRotation', 0);
    xlabel('Grid x');
    ylabel('Grid y');

    drawGridObstacles(gridMap);

    if ~isempty(obsUnknown)
        for i = 1:size(obsUnknown, 1)
            x = obsUnknown(i, 1);
            y = obsUnknown(i, 2);
            fill([x x+1 x+1 x], [y y y+1 y+1], [0.5 0.5 0.5], 'EdgeColor', 'k');
        end
    end

    if ~isempty(moveobsTrajectory)
        plot(moveobsTrajectory(:,1)+0.5, moveobsTrajectory(:,2)+0.5, ...
            'Color', [0.4 0.4 0.4], 'LineStyle', ':', 'LineWidth', 1.2);
    end

    hGlobal = plot(globalPath(:,1)+0.5, globalPath(:,2)+0.5, ...
        'b:', 'LineWidth', 2);
    hNoLOS = plot(pathNoLOS(:,1)+0.5, pathNoLOS(:,2)+0.5, ...
        'r--', 'LineWidth', 2);
    hLOS = plot(pathLOS(:,1)+0.5, pathLOS(:,2)+0.5, ...
        'b-', 'LineWidth', 2);

    hStart = plot(start(1)+0.5, start(2)+0.5, 'b^', ...
        'LineWidth', 2, 'MarkerSize', 8);
    hGoal = plot(goal(1)+0.5, goal(2)+0.5, 'go', ...
        'LineWidth', 2, 'MarkerSize', 8);
    text(start(1)+0.7, start(2)+0.5, 'Start', 'FontSize', 12);
    text(goal(1)+0.7, goal(2)+0.5, 'End', 'FontSize', 12);

    legend([hGlobal, hNoLOS, hLOS, hStart, hGoal], ...
        {'IA* + Floyd path', 'IA* + Floyd + DWA', ...
         'IA* + Floyd + LOS + DWA', 'Start', 'End'}, ...
         'Location', 'best');
    title('Final LOS ablation trajectory comparison');
end

function drawGridObstacles(gridMap)
    [obsRows, obsCols] = find(gridMap == 1);
    for i = 1:numel(obsRows)
        fill([obsRows(i), obsRows(i)+1, obsRows(i)+1, obsRows(i)], ...
             [obsCols(i), obsCols(i), obsCols(i)+1, obsCols(i)+1], ...
             'k', 'EdgeColor', 'none');
    end
end

function radian = toRadian(degree)
    radian = degree / 180 * pi;
end
