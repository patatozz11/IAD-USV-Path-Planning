clc;
clear;
close all;

% Full LOS parameter sensitivity test.
%
% This script evaluates all combinations of:
%   sight_dist = [3.0, 4.0, 5.0, 6.0]
%   max_skip   = [3, 5, 8]
%   d_safe     = [0.2, 0.3, 0.4]
%
% It calls the final root-level DWA_Fusion_noLOS and DWA_Fusion_LOS
% functions only. It does not implement simplified DWA/LOS internally.

batchMode = true;
outDir = fullfile('results', 'los_parameter_sensitivity_full_corrected_distance');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

resultsCsv = fullfile(outDir, 'los_parameter_sensitivity_full_results.csv');
resultsMat = fullfile(outDir, 'los_parameter_sensitivity_full_results.mat');
baselineCsv = fullfile(outDir, 'los_parameter_sensitivity_full_noLOS_baseline.csv');
logFile = fullfile(outDir, 'run_log.txt');

oldFigureVisible = get(0, 'DefaultFigureVisible');
if batchMode
    set(0, 'DefaultFigureVisible', 'off');
    diary(logFile);
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
Obs_Unknown = [];
moveobs_trajectory = [];
diagRule = 1;

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

%% noLOS baseline, run once and save
if isfile(baselineCsv)
    baselineTable = readtable(baselineCsv, 'TextType', 'string');
    fprintf('\nLoaded existing noLOS baseline from %s\n', baselineCsv);
else
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
        'minimum_obstacle_distance', 'average_obstacle_distance', ...
        'cumulative_angular_velocity_change'});
    writetable(baselineTable, baselineCsv);
end

baseline = tableToBaselineStruct(baselineTable);

%% Parameter combinations
sight_dist_list = [3.0, 4.0, 5.0, 6.0];
max_skip_list   = [3, 5, 8];
d_safe_list     = [0.2, 0.3, 0.4];

if isfile(resultsCsv)
    resultsTable = readtable(resultsCsv);
    fprintf('\nLoaded existing partial results: %d rows\n', height(resultsTable));
else
    resultsTable = emptyResultsTable();
end

totalRuns = numel(sight_dist_list) * numel(max_skip_list) * numel(d_safe_list);
runCounter = 0;

for iSight = 1:numel(sight_dist_list)
    for iSkip = 1:numel(max_skip_list)
        for iSafe = 1:numel(d_safe_list)
            runCounter = runCounter + 1;
            sight_dist = sight_dist_list(iSight);
            max_skip = max_skip_list(iSkip);
            d_safe = d_safe_list(iSafe);

            if isCompleted(resultsTable, sight_dist, max_skip, d_safe)
                fprintf('\n[%02d/%02d] Skip completed: sight=%.1f, max_skip=%d, d_safe=%.1f\n', ...
                    runCounter, totalRuns, sight_dist, max_skip, d_safe);
                continue;
            end

            fprintf('\n[%02d/%02d] Running LOS: sight=%.1f, max_skip=%d, d_safe=%.1f\n', ...
                runCounter, totalRuns, sight_dist, max_skip, d_safe);

            losConfig.reach_dist = 1.0;
            losConfig.sight_dist = sight_dist;
            losConfig.max_skip = max_skip;

            clear DWA_Fusion_LOS
            prepareBatchFigure(batchMode);
            [Final_Path_LOS, metrics_LOS] = DWA_Fusion_LOS( ...
                gridMap, obs_Static, Obs_Unknown, moveobs_trajectory, Path, ...
                start, goal, Kinematic, evalParam, d_safe, losConfig);
            close all;

            newRow = table( ...
                sight_dist, max_skip, d_safe, metrics_LOS.success, ...
                metrics_LOS.path_length, metrics_LOS.navigation_time, ...
                metrics_LOS.minimum_obstacle_distance, metrics_LOS.average_obstacle_distance, ...
                metrics_LOS.cumulative_delta_angular_velocity, getOptionalMetric(metrics_LOS, 'target_switches'), ...
                getOptionalMetric(metrics_LOS, 'LOS_skip_count'), getOptionalMetric(metrics_LOS, 'avg_local_goal_distance'), ...
                getOptionalMetric(metrics_LOS, 'mean_tracking_error_to_global_path'), ...
                'VariableNames', resultsTable.Properties.VariableNames);

            resultsTable = [resultsTable; newRow]; %#ok<AGROW>
            writetable(resultsTable, resultsCsv);
            save(resultsMat, 'resultsTable', 'baselineTable', 'Path', 'gridMap', ...
                'start', 'goal', 'Obs_Unknown', 'moveobs_trajectory', 'Kinematic', ...
                'evalParam', 'diagRule', 'distanceX', 'OPEN_num', 'run_time');
        end
    end
end

%% Recommendation and final outputs
topTable = recommendTopParameters(resultsTable, baseline);
best = topTable(1, :);

fprintf('\nShared IA* + Floyd path:\n');
fprintf('  Path length: %.3f | nodes: %d | OPEN nodes: %d | planning time: %.3f ms\n', ...
    distanceX, size(Path, 1), OPEN_num, run_time);

fprintf('\nnoLOS baseline:\n');
disp(baselineTable);

fprintf('\nFull LOS parameter sensitivity table:\n');
disp(resultsTable);

fprintf('\nTop 5 recommended LOS parameter sets:\n');
disp(topTable(1:min(5, height(topTable)), :));

fprintf('\nRecommended final parameters:\n');
fprintf('  sight_dist = %.1f\n', best.sight_dist);
fprintf('  max_skip   = %d\n', best.max_skip);
fprintf('  d_safe     = %.1f\n', best.d_safe);

save(resultsMat, 'resultsTable', 'baselineTable', 'topTable', 'Path', 'gridMap', ...
    'start', 'goal', 'Obs_Unknown', 'moveobs_trajectory', 'Kinematic', ...
    'evalParam', 'diagRule', 'distanceX', 'OPEN_num', 'run_time');

%% Save trajectory comparison only for the best parameter set
losConfig.reach_dist = 1.0;
losConfig.sight_dist = best.sight_dist;
losConfig.max_skip = best.max_skip;
clear DWA_Fusion_noLOS
prepareBatchFigure(batchMode);
[Final_Path_noLOS, ~] = DWA_Fusion_noLOS( ...
    gridMap, obs_Static, Obs_Unknown, moveobs_trajectory, Path, start, goal, ...
    Kinematic, evalParam);
close all;

clear DWA_Fusion_LOS
prepareBatchFigure(batchMode);
[Final_Path_BestLOS, ~] = DWA_Fusion_LOS( ...
    gridMap, obs_Static, Obs_Unknown, moveobs_trajectory, Path, start, goal, ...
    Kinematic, evalParam, best.d_safe, losConfig);
close all;

plotBestSensitivityResult(gridMap, Path, Final_Path_noLOS, Final_Path_BestLOS, ...
    start, goal, best);
saveas(gcf, fullfile(outDir, 'best_los_parameter_trajectory_comparison.fig'));
print(gcf, fullfile(outDir, 'best_los_parameter_trajectory_comparison.png'), '-dpng', '-r600');

fprintf('\nSaved full sensitivity results to: %s\n', outDir);

%% Helper functions
function T = emptyResultsTable()
    T = table( ...
        zeros(0,1), zeros(0,1), zeros(0,1), false(0,1), ...
        zeros(0,1), zeros(0,1), zeros(0,1), zeros(0,1), zeros(0,1), ...
        NaN(0,1), NaN(0,1), NaN(0,1), NaN(0,1), ...
        'VariableNames', {'sight_dist', 'max_skip', 'd_safe', 'success', ...
        'path_length', 'navigation_time', 'minimum_obstacle_distance', ...
        'average_obstacle_distance', 'cumulative_angular_velocity_change', ...
        'target_switches', 'LOS_skip_count', 'avg_local_goal_distance', ...
        'mean_tracking_error_to_global_path'});
end

function tf = isCompleted(T, sight_dist, max_skip, d_safe)
    if isempty(T)
        tf = false;
        return;
    end
    tf = any(abs(T.sight_dist - sight_dist) < 1e-9 & ...
             T.max_skip == max_skip & ...
             abs(T.d_safe - d_safe) < 1e-9);
end

function value = getOptionalMetric(metrics, fieldName)
    if isfield(metrics, fieldName)
        value = metrics.(fieldName);
    else
        value = NaN;
    end
end

function baseline = tableToBaselineStruct(T)
    baseline.path_length = T.path_length(1);
    baseline.navigation_time = T.navigation_time(1);
    baseline.minimum_obstacle_distance = T.minimum_obstacle_distance(1);
    baseline.average_obstacle_distance = T.average_obstacle_distance(1);
    baseline.cumulative_angular_velocity_change = T.cumulative_angular_velocity_change(1);
end

function topTable = recommendTopParameters(T, baseline)
    successful = T(T.success == true, :);
    if isempty(successful)
        topTable = T;
        return;
    end

    lengthLimit = baseline.path_length * 1.05;
    timeLimit = baseline.navigation_time * 1.05;
    avgDistLimit = baseline.average_obstacle_distance * 0.95;

    feasible = successful.path_length <= lengthLimit & ...
               successful.navigation_time <= timeLimit & ...
               successful.minimum_obstacle_distance >= baseline.minimum_obstacle_distance - 1e-9 & ...
               successful.average_obstacle_distance >= avgDistLimit;

    successful.feasible = feasible;
    % Sort by feasibility first, then smoothness, then conservative choices:
    % smaller max_skip, larger d_safe, and smaller sight_dist.
    successful.neg_feasible = -double(successful.feasible);
    successful.neg_d_safe = -successful.d_safe;
    topTable = sortrows(successful, {'neg_feasible', ...
        'cumulative_angular_velocity_change', 'max_skip', 'neg_d_safe', 'sight_dist'});
    topTable.neg_feasible = [];
    topTable.neg_d_safe = [];
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
