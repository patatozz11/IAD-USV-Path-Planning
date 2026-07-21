clc;
clear;
close all;

% LOS parameter sensitivity test.
%
% This script only changes LOS parameters in DWA_Fusion_LOS through
% losConfig and d_safe. It does not modify DWA evaluation, Kinematic,
% IA*/A*, map logic, or the final fusion functions.

batchMode = true;
stageMode = "stage1";  % "stage1", "stage2", or "full"
outDir = fullfile('results', 'los_parameter_sensitivity');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

oldFigureVisible = get(0, 'DefaultFigureVisible');
if batchMode
    set(0, 'DefaultFigureVisible', 'off');
    diary(fullfile(outDir, 'run_log.txt'));
    diary on;
end

cleanupObj = onCleanup(@() restoreFigureAndDiary(batchMode, oldFigureVisible));

disp('Function path check:');
disp(['DWA_Fusion_noLOS: ', which('DWA_Fusion_noLOS')]);
disp(['DWA_Fusion_LOS  : ', which('DWA_Fusion_LOS')]);
disp(['DynamicWindowApproach: ', which('DynamicWindowApproach')]);
disp(['Evaluation: ', which('Evaluation')]);
disp(['IAstar_FiveDir_Fallback_Fast: ', which('IAstar_FiveDir_Fallback_Fast')]);
disp(['Floyd_Smooth_safer: ', which('Floyd_Smooth_safer')]);

%% Shared experiment settings
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

start = [2, 2];
goal  = [19, 18];
diagRule = 1;

% Parameter sensitivity uses the same no-dynamic/no-unknown condition as
% the current reproducible LOS ablation script.
Obs_Unknown = [];
moveobs_trajectory = [];

global dt;
dt = 0.1;
Kinematic = [1.0, toRadian(20.0), 0.2, toRadian(50.0), 0.01, toRadian(1)];
evalParam = [0.05, 0.2, 0.1, 3.0];

[Path, distanceX, OPEN_num, ~, run_time] = ...
    IAstar_FiveDir_Fallback_Fast(gridMap, start, goal, diagRule);
if isempty(Path)
    error('IA* failed to generate the shared global path.');
end

[obs_static_rows, obs_static_cols] = find(gridMap == 1);
obs_Static = [obs_static_rows, obs_static_cols];

%% Baseline noLOS run
fprintf('\n===== Running noLOS baseline =====\n');
clear DWA_Fusion_noLOS
prepareBatchFigure(batchMode);
[Final_Path_noLOS, metrics_noLOS] = DWA_Fusion_noLOS( ...
    gridMap, obs_Static, Obs_Unknown, moveobs_trajectory, Path, start, goal, ...
    Kinematic, evalParam);
close all;

baselineTable = table( ...
    "IA*+Floyd+DWA", metrics_noLOS.success, metrics_noLOS.path_length, ...
    metrics_noLOS.navigation_time, metrics_noLOS.minimum_obstacle_distance, ...
    metrics_noLOS.average_obstacle_distance, ...
    metrics_noLOS.cumulative_delta_angular_velocity, ...
    'VariableNames', {'Method', 'success', 'path_length', 'navigation_time', ...
    'min_obstacle_distance', 'avg_obstacle_distance', ...
    'cumulative_angular_velocity_change'});

%% LOS parameter combinations
switch lower(stageMode)
    case "stage1"
        sight_dist_list = [3.0, 4.0, 5.0, 6.0];
        max_skip_list = 5;
        d_safe_list = 0.2;
    case "stage2"
        sight_dist_list = 6.0;
        max_skip_list = [3, 5, 8];
        d_safe_list = [0.2, 0.3, 0.4];
    case "full"
        sight_dist_list = [3.0, 4.0, 5.0, 6.0];
        max_skip_list = [3, 5, 8];
        d_safe_list = [0.2, 0.3, 0.4];
    otherwise
        error('Unknown stageMode: %s', stageMode);
end

rows = [];
rowCount = 0;
bestMetrics = [];
bestFinalPath = [];

for iSight = 1:numel(sight_dist_list)
    for iSkip = 1:numel(max_skip_list)
        for iSafe = 1:numel(d_safe_list)
            sight_dist = sight_dist_list(iSight);
            max_skip = max_skip_list(iSkip);
            d_safe = d_safe_list(iSafe);

            losConfig.reach_dist = 1.0;
            losConfig.sight_dist = sight_dist;
            losConfig.max_skip = max_skip;

            fprintf('\n===== Running LOS: sight_dist=%.1f, max_skip=%d, d_safe=%.1f =====\n', ...
                sight_dist, max_skip, d_safe);

            clear DWA_Fusion_LOS
            prepareBatchFigure(batchMode);
            [Final_Path_LOS, metrics_LOS] = DWA_Fusion_LOS( ...
                gridMap, obs_Static, Obs_Unknown, moveobs_trajectory, Path, ...
                start, goal, Kinematic, evalParam, d_safe, losConfig);
            close all;

            rowCount = rowCount + 1;
            rows(rowCount).sight_dist = sight_dist; %#ok<SAGROW>
            rows(rowCount).max_skip = max_skip;
            rows(rowCount).d_safe = d_safe;
            rows(rowCount).success = metrics_LOS.success;
            rows(rowCount).path_length = metrics_LOS.path_length;
            rows(rowCount).navigation_time = metrics_LOS.navigation_time;
            rows(rowCount).min_obstacle_distance = metrics_LOS.minimum_obstacle_distance;
            rows(rowCount).avg_obstacle_distance = metrics_LOS.average_obstacle_distance;
            rows(rowCount).cumulative_angular_velocity_change = metrics_LOS.cumulative_delta_angular_velocity;

            if isempty(bestMetrics) || isBetterParameterSet(rows(rowCount), bestMetrics, metrics_noLOS)
                bestMetrics = rows(rowCount);
                bestFinalPath = Final_Path_LOS;
            end
        end
    end
end

resultsTable = struct2table(rows);

%% Save outputs
writetable(resultsTable, fullfile(outDir, 'los_parameter_sensitivity_results.csv'));
writetable(baselineTable, fullfile(outDir, 'los_parameter_sensitivity_noLOS_baseline.csv'));
save(fullfile(outDir, 'los_parameter_sensitivity_results.mat'), ...
    'resultsTable', 'baselineTable', 'metrics_noLOS', 'Path', ...
    'Final_Path_noLOS', 'bestFinalPath', 'bestMetrics', 'gridMap', ...
    'start', 'goal', 'Obs_Unknown', 'moveobs_trajectory', 'Kinematic', ...
    'evalParam', 'stageMode', 'distanceX', 'OPEN_num', 'run_time');

fprintf('\nShared IA* + Floyd path:\n');
fprintf('  Path length: %.3f | nodes: %d | OPEN nodes: %d | planning time: %.3f ms\n', ...
    distanceX, size(Path, 1), OPEN_num, run_time);

fprintf('\nnoLOS baseline:\n');
disp(baselineTable);

fprintf('\nLOS parameter sensitivity results:\n');
disp(resultsTable);

fprintf('\nRecommended LOS parameter set in this run:\n');
fprintf('  sight_dist = %.1f\n', bestMetrics.sight_dist);
fprintf('  max_skip   = %d\n', bestMetrics.max_skip);
fprintf('  d_safe     = %.1f\n', bestMetrics.d_safe);
fprintf('  cumulative angular velocity change = %.5f\n', ...
    bestMetrics.cumulative_angular_velocity_change);
fprintf('  path length = %.3f, navigation time = %.3f, min obstacle distance = %.3f\n', ...
    bestMetrics.path_length, bestMetrics.navigation_time, ...
    bestMetrics.min_obstacle_distance);

if ~isempty(bestFinalPath)
    plotBestSensitivityResult(gridMap, Path, Final_Path_noLOS, bestFinalPath, ...
        start, goal, bestMetrics);
    saveas(gcf, fullfile(outDir, 'best_los_parameter_trajectory_comparison.fig'));
    print(gcf, fullfile(outDir, 'best_los_parameter_trajectory_comparison.png'), '-dpng', '-r600');
end

fprintf('\nSaved sensitivity results to: %s\n', outDir);

%% Helper functions
function tf = isBetterParameterSet(candidate, currentBest, baseline)
    if ~candidate.success
        tf = false;
        return;
    end
    if ~currentBest.success
        tf = true;
        return;
    end

    safetyTol = 1e-9;
    lengthLimit = baseline.path_length * 1.05;
    timeLimit = baseline.navigation_time * 1.05;
    safeEnough = candidate.min_obstacle_distance >= baseline.minimum_obstacle_distance - safetyTol;
    notLonger = candidate.path_length <= lengthLimit;
    notSlower = candidate.navigation_time <= timeLimit;

    currentSafeEnough = currentBest.min_obstacle_distance >= baseline.minimum_obstacle_distance - safetyTol;
    currentNotLonger = currentBest.path_length <= lengthLimit;
    currentNotSlower = currentBest.navigation_time <= timeLimit;

    candidateFeasible = safeEnough && notLonger && notSlower;
    currentFeasible = currentSafeEnough && currentNotLonger && currentNotSlower;

    if candidateFeasible && ~currentFeasible
        tf = true;
    elseif candidateFeasible == currentFeasible
        tf = candidate.cumulative_angular_velocity_change < currentBest.cumulative_angular_velocity_change;
    else
        tf = false;
    end
end

function prepareBatchFigure(batchMode)
    if batchMode
        close all;
        figure('Visible', 'off', 'Color', 'w');
        hold on;
        axis equal;
    end
end

function plotBestSensitivityResult(gridMap, globalPath, pathNoLOS, pathLOS, start, goal, bestMetrics)
    [maxRow, maxCol] = size(gridMap);
    figure('Visible', 'off', 'Color', 'w');
    hold on;
    axis equal;
    axis([1 maxRow+1, 1 maxCol+1]);
    set(gca, 'XTick', 1:maxRow+1, 'YTick', 1:maxCol+1, ...
        'XGrid', 'on', 'YGrid', 'on', 'GridLineStyle', '-', ...
        'XTickLabelRotation', 0);
    xlabel('Grid x');
    ylabel('Grid y');

    [obsRows, obsCols] = find(gridMap == 1);
    for ii = 1:numel(obsRows)
        fill([obsRows(ii), obsRows(ii)+1, obsRows(ii)+1, obsRows(ii)], ...
             [obsCols(ii), obsCols(ii), obsCols(ii)+1, obsCols(ii)+1], ...
             'k', 'EdgeColor', 'none');
    end

    hGlobal = plot(globalPath(:,1)+0.5, globalPath(:,2)+0.5, 'b:', 'LineWidth', 2);
    hNoLOS = plot(pathNoLOS(:,1)+0.5, pathNoLOS(:,2)+0.5, 'r--', 'LineWidth', 2);
    hLOS = plot(pathLOS(:,1)+0.5, pathLOS(:,2)+0.5, 'b-', 'LineWidth', 2);
    hStart = plot(start(1)+0.5, start(2)+0.5, 'b^', 'LineWidth', 2, 'MarkerSize', 8);
    hGoal = plot(goal(1)+0.5, goal(2)+0.5, 'go', 'LineWidth', 2, 'MarkerSize', 8);
    text(start(1)+0.7, start(2)+0.5, 'Start', 'FontSize', 12);
    text(goal(1)+0.7, goal(2)+0.5, 'End', 'FontSize', 12);

    legend([hGlobal, hNoLOS, hLOS, hStart, hGoal], ...
        {'IA* + Floyd path', 'IA* + Floyd + DWA', ...
         'Best IA* + Floyd + LOS + DWA', 'Start', 'End'}, ...
        'Location', 'best');
    title(sprintf('Best LOS parameters: sight=%.1f, skip=%d, d_{safe}=%.1f', ...
        bestMetrics.sight_dist, bestMetrics.max_skip, bestMetrics.d_safe));
end

function restoreFigureAndDiary(batchMode, oldFigureVisible)
    if batchMode
        diary off;
        set(0, 'DefaultFigureVisible', oldFigureVisible);
    end
end

function radian = toRadian(degree)
    radian = degree / 180 * pi;
end
