%% compare_LOS_safeWidth_vs_dSafe
% This script compares two LOS safety-check modes:
%   A. safe_width / isLineSafe LOS check in DWA_improvedFusion
%   B. d_safe clearance LOS check in DWA_Fusion_LOS_Traditional
%
% This is NOT a with-LOS / without-LOS ablation experiment.
% This is NOT a traditional-DWA / improved-DWA experiment.
% Both methods use LOS and both methods use the same traditional DWA.
% The only intended difference is the LOS visibility safety check.

clc;
clear;
close all;
rng(1);

%% Output folder
resultDir = fullfile(pwd, 'results', 'los_safety_mode_comparison');
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end

%% Verification of function resolution
disp('========== Function resolution ==========');
disp(['DWA_improvedFusion:          ', which('DWA_improvedFusion')]);
disp(['DWA_Fusion_LOS_Traditional:  ', which('DWA_Fusion_LOS_Traditional')]);
disp(['DynamicWindowApproach:       ', which('DynamicWindowApproach')]);
disp(['Evaluation:                  ', which('Evaluation')]);
disp(['IAstar_FiveDir_Fallback_Fast:', which('IAstar_FiveDir_Fallback_Fast')]);
disp(['Floyd_Smooth_safer:          ', which('Floyd_Smooth_safer')]);

%% Shared DWA parameters
global dt;
dt = 0.1;
Kinematic = [1.0, deg2rad_local(20.0), 0.2, deg2rad_local(50.0), 0.01, deg2rad_local(1.0)];
evalParam = [0.05, 0.2, 0.1, 3.0];  % [heading, obstacle distance, velocity, predict time]
diagRule = 1;

dSafeList = [0.1, 0.2, 0.25, 0.3, 0.5];
keyDSafeForFigures = 0.2;

mapCases = createMapCases();
allRows = {};

oldFigVisible = get(0, 'DefaultFigureVisible');
set(0, 'DefaultFigureVisible', 'off');
cleanupObj = onCleanup(@() set(0, 'DefaultFigureVisible', oldFigVisible));

for mapIdx = 1:numel(mapCases)
    mapCase = mapCases(mapIdx);
    gridMap = mapCase.gridMap;
    start = mapCase.start;
    goal = mapCase.goal;
    Obs_Unknown = mapCase.unknownObstacles;
    moveobs_trajectory = mapCase.movingObstacleTrajectory;
    [obsRows, obsCols] = find(gridMap == 1);
    obs_Static = [obsRows, obsCols];

    fprintf('\n========== LOS Safety Mode Comparison ==========\n');
    fprintf('Map: %s\n', mapCase.name);

    if gridMap(start(1), start(2)) == 1 || gridMap(goal(1), goal(2)) == 1
        warning('Start or goal is on an obstacle in %s. Skipping this map.', mapCase.name);
        continue;
    end

    % Generate IA* + Floyd path once only. IAstar_FiveDir_Fallback_Fast already
    % returns the Floyd-smoothed path, so do not call Floyd_Smooth_safer again.
    [Path, distanceX, OPEN_num, ~, run_time] = IAstar_FiveDir_Fallback_Fast(gridMap, start, goal, diagRule);
    if isempty(Path)
        warning('IA* did not find a path in %s. Skipping this map.', mapCase.name);
        continue;
    end
    fprintf('Shared IA*+Floyd path: length %.3f, OPEN %d, time %.3f ms\n', distanceX, OPEN_num, run_time);

    clear DWA_improvedFusion;
    figOld = figure('Visible', 'off'); hold on; axis equal;
    [Final_Path_old, metrics_old] = DWA_improvedFusion(gridMap, obs_Static, Obs_Unknown, moveobs_trajectory, Path, start, goal, Kinematic, evalParam);
    close(figOld);

    for dIdx = 1:numel(dSafeList)
        d_safe = dSafeList(dIdx);

        clear DWA_Fusion_LOS_Traditional;
        figNew = figure('Visible', 'off'); hold on; axis equal;
        [Final_Path_new, metrics_new] = DWA_Fusion_LOS_Traditional(gridMap, obs_Static, Obs_Unknown, moveobs_trajectory, Path, start, goal, Kinematic, evalParam, d_safe);
        close(figNew);

        printComparison(mapCase.name, d_safe, metrics_old, metrics_new);
        allRows = appendMetricRows(allRows, mapCase.name, d_safe, metrics_old, metrics_new);

        if abs(d_safe - keyDSafeForFigures) < 1e-12
            saveComparisonFigures(resultDir, mapCase.name, d_safe, gridMap, start, goal, Path, Final_Path_old, Final_Path_new, metrics_old, metrics_new, dt);
        end
    end
end

metricsTable = cell2table(allRows, 'VariableNames', { ...
    'MapName', 'DSafe', 'Method', 'PathLength', 'NavigationTime', 'ComputationTime', ...
    'MinimumObstacleDistance', 'AverageObstacleDistance', 'MaxDeltaAngularVelocity', ...
    'CumulativeDeltaAngularVelocity', 'MeanDeltaAngularVelocity', 'RMSDeltaAngularVelocity', ...
    'MaxHeadingChange', 'CumulativeHeadingChange', 'TargetSwitches', 'Success'});

csvPath = fullfile(resultDir, 'los_safety_mode_metrics.csv');
writetable(metricsTable, csvPath);
fprintf('\nSaved metrics CSV:\n%s\n', csvPath);

printOverallJudgement(metricsTable);

%% Local helper functions
function mapCases = createMapCases()
    baseMap = [
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
    baseMap = rot90(baseMap, 3);

    denseMap = baseMap;
    denseMap(6,8) = 1; denseMap(7,8) = 1; denseMap(12,13) = 1;
    denseMap(13,13) = 1; denseMap(15,9) = 1; denseMap(16,10) = 1;

    corridorMap = zeros(14,14);
    corridorMap(4:11,4) = 1;
    corridorMap(4,4:10) = 1;
    corridorMap(4:10,10) = 1;
    corridorMap(10,7:10) = 1;
    corridorMap(7,6:9) = 1;
    corridorMap(9,2:5) = 1;
    corridorMap(2,11:12) = 1;
    corridorMap(12,11:12) = 1;

    mapCases = struct([]);
    mapCases(1).name = 'regular_static_map';
    mapCases(1).gridMap = baseMap;
    mapCases(1).start = [2, 2];
    mapCases(1).goal = [20, 19];
    mapCases(1).unknownObstacles = [];
    mapCases(1).movingObstacleTrajectory = [];

    mapCases(2).name = 'dense_static_map';
    mapCases(2).gridMap = denseMap;
    mapCases(2).start = [2, 2];
    mapCases(2).goal = [20, 19];
    mapCases(2).unknownObstacles = [];
    mapCases(2).movingObstacleTrajectory = [];

    mapCases(3).name = 'corner_channel_map';
    mapCases(3).gridMap = corridorMap;
    mapCases(3).start = [2, 2];
    mapCases(3).goal = [13, 13];
    mapCases(3).unknownObstacles = [];
    mapCases(3).movingObstacleTrajectory = [];
end

function rows = appendMetricRows(rows, mapName, d_safe, metrics_old, metrics_new)
    rows(end+1,:) = metricRow(mapName, d_safe, 'safe_width_isLineSafe', metrics_old);
    rows(end+1,:) = metricRow(mapName, d_safe, 'd_safe_clearance', metrics_new);
end

function row = metricRow(mapName, d_safe, method, metrics)
    row = {mapName, d_safe, method, metrics.path_length, metrics.navigation_time, metrics.computation_time, ...
        metrics.minimum_obstacle_distance, metrics.average_obstacle_distance, ...
        metrics.max_delta_angular_velocity, metrics.cumulative_delta_angular_velocity, ...
        metrics.mean_delta_angular_velocity, metrics.RMS_delta_angular_velocity, ...
        metrics.max_heading_change, metrics.cumulative_heading_change, ...
        metrics.target_switches, double(metrics.success)};
end

function printComparison(mapName, d_safe, oldMetrics, newMetrics)
    fprintf('\n========== LOS Safety Mode Comparison ==========\n');
    fprintf('Map: %s\n', mapName);
    fprintf('d_safe: %.2f\n\n', d_safe);
    fprintf('%-28s %8s %8s %8s %8s %8s %9s %9s %9s %9s %9s %9s %9s %8s\n', ...
        'Method', 'PathLen', 'NavTime', 'CompTime', 'MinObs', 'AvgObs', ...
        'MaxDw', 'SumDw', 'MeanDw', 'RMSDw', 'MaxDTh', 'SumDTh', 'Switches', 'Success');
    printMetricLine('safe_width / isLineSafe', oldMetrics);
    printMetricLine('d_safe clearance', newMetrics);
    printDetailedMetrics('safe_width / isLineSafe', oldMetrics);
    printDetailedMetrics('d_safe clearance', newMetrics);
end

function printMetricLine(name, m)
    fprintf('%-28s %8.2f %8.2f %8.2f %8.2f %8.2f %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f %9d %8d\n', ...
        name, m.path_length, m.navigation_time, m.computation_time, ...
        m.minimum_obstacle_distance, m.average_obstacle_distance, ...
        m.max_delta_angular_velocity, m.cumulative_delta_angular_velocity, ...
        m.mean_delta_angular_velocity, m.RMS_delta_angular_velocity, ...
        m.max_heading_change, m.cumulative_heading_change, ...
        m.target_switches, double(m.success));
end

function printDetailedMetrics(name, m)
    fprintf('\n[%s]\n', name);
    fprintf('Actual path length: %.3f m\n', m.path_length);
    fprintf('Navigation time: %.3f s\n', m.navigation_time);
    fprintf('Computation time: %.3f s\n', m.computation_time);
    fprintf('Minimum obstacle distance: %.3f m\n', m.minimum_obstacle_distance);
    fprintf('Average obstacle distance: %.3f m\n', m.average_obstacle_distance);
    fprintf('Max delta angular velocity: %.5f rad/s\n', m.max_delta_angular_velocity);
    fprintf('Cumulative delta angular velocity: %.5f rad/s\n', m.cumulative_delta_angular_velocity);
    fprintf('Mean delta angular velocity: %.5f rad/s\n', m.mean_delta_angular_velocity);
    fprintf('RMS delta angular velocity: %.5f rad/s\n', m.RMS_delta_angular_velocity);
    fprintf('Maximum heading change: %.5f rad\n', m.max_heading_change);
    fprintf('Cumulative heading change: %.5f rad\n', m.cumulative_heading_change);
    fprintf('LOS target switches: %d\n', m.target_switches);
    fprintf('Success: %d\n', double(m.success));
end

function saveComparisonFigures(resultDir, mapName, d_safe, gridMap, start, goal, refPath, oldPath, newPath, oldMetrics, newMetrics, dt)
    safeName = matlab.lang.makeValidName(sprintf('%s_dsafe_%0.2f', mapName, d_safe));

    fig = figure('Color', 'w', 'Visible', 'off');
    plotGridMap(gridMap); hold on;
    plot(refPath(:,1)+0.5, refPath(:,2)+0.5, 'k--', 'LineWidth', 1.5);
    plot(oldPath(:,1)+0.5, oldPath(:,2)+0.5, 'r-', 'LineWidth', 1.8);
    plot(newPath(:,1)+0.5, newPath(:,2)+0.5, 'b-', 'LineWidth', 1.8);
    plot(start(1)+0.5, start(2)+0.5, 'b^', 'MarkerSize', 8, 'LineWidth', 1.5);
    plot(goal(1)+0.5, goal(2)+0.5, 'go', 'MarkerSize', 8, 'LineWidth', 1.5);
    legend({'IA*+Floyd path', 'safe\_width/isLineSafe', 'd\_safe clearance', 'Start', 'Goal'}, 'Location', 'best');
    title(sprintf('%s, d\\_safe = %.2f', mapName, d_safe), 'Interpreter', 'none');
    saveas(fig, fullfile(resultDir, [safeName, '_trajectory.png']));
    savefig(fig, fullfile(resultDir, [safeName, '_trajectory.fig']));
    close(fig);

    fig = figure('Color', 'w', 'Visible', 'off');
    plotMetricHistory(oldMetrics.angular_velocity_history, dt, 'r-', 'safe\_width/isLineSafe'); hold on;
    plotMetricHistory(newMetrics.angular_velocity_history, dt, 'b-', 'd\_safe clearance');
    xlabel('Time (s)'); ylabel('Angular velocity (rad/s)'); grid on;
    title(sprintf('Angular velocity comparison: %s', mapName), 'Interpreter', 'none');
    legend('Location', 'best');
    saveas(fig, fullfile(resultDir, [safeName, '_angular_velocity.png']));
    savefig(fig, fullfile(resultDir, [safeName, '_angular_velocity.fig']));
    close(fig);

    fig = figure('Color', 'w', 'Visible', 'off');
    plotMetricHistory(oldMetrics.obstacle_distance_history, dt, 'r-', 'safe\_width/isLineSafe'); hold on;
    plotMetricHistory(newMetrics.obstacle_distance_history, dt, 'b-', 'd\_safe clearance');
    xlabel('Time (s)'); ylabel('Obstacle boundary distance (grid)'); grid on;
    title(sprintf('Obstacle distance comparison: %s', mapName), 'Interpreter', 'none');
    legend('Location', 'best');
    saveas(fig, fullfile(resultDir, [safeName, '_obstacle_distance.png']));
    savefig(fig, fullfile(resultDir, [safeName, '_obstacle_distance.fig']));
    close(fig);
end

function plotGridMap(gridMap)
    [R, C] = size(gridMap);
    axis equal;
    axis([1 R+1 1 C+1]);
    set(gca, 'XTick', 1:R+1, 'YTick', 1:C+1, 'XGrid', 'on', 'YGrid', 'on');
    for i = 1:R
        for j = 1:C
            if gridMap(i,j) == 1
                fill([i i+1 i+1 i], [j j j+1 j+1], 'k', 'EdgeColor', 'none');
            end
        end
    end
end

function plotMetricHistory(values, dt, lineSpec, displayName)
    if isempty(values)
        plot(NaN, NaN, lineSpec, 'DisplayName', displayName);
        return;
    end
    t = (0:numel(values)-1) * dt;
    plot(t, values, lineSpec, 'LineWidth', 1.5, 'DisplayName', displayName);
end

function printOverallJudgement(metricsTable)
    oldRows = strcmp(metricsTable.Method, 'safe_width_isLineSafe');
    newRows = strcmp(metricsTable.Method, 'd_safe_clearance');
    if ~any(oldRows) || ~any(newRows)
        return;
    end

    oldMin = mean(metricsTable.MinimumObstacleDistance(oldRows), 'omitnan');
    newMin = mean(metricsTable.MinimumObstacleDistance(newRows), 'omitnan');
    oldAvg = mean(metricsTable.AverageObstacleDistance(oldRows), 'omitnan');
    newAvg = mean(metricsTable.AverageObstacleDistance(newRows), 'omitnan');
    oldLen = mean(metricsTable.PathLength(oldRows), 'omitnan');
    newLen = mean(metricsTable.PathLength(newRows), 'omitnan');
    oldTime = mean(metricsTable.NavigationTime(oldRows), 'omitnan');
    newTime = mean(metricsTable.NavigationTime(newRows), 'omitnan');
    oldRmsDw = mean(metricsTable.RMSDeltaAngularVelocity(oldRows), 'omitnan');
    newRmsDw = mean(metricsTable.RMSDeltaAngularVelocity(newRows), 'omitnan');
    oldMaxHeading = mean(metricsTable.MaxHeadingChange(oldRows), 'omitnan');
    newMaxHeading = mean(metricsTable.MaxHeadingChange(newRows), 'omitnan');
    oldCumHeading = mean(metricsTable.CumulativeHeadingChange(oldRows), 'omitnan');
    newCumHeading = mean(metricsTable.CumulativeHeadingChange(newRows), 'omitnan');
    successNew = all(metricsTable.Success(newRows) == 1);

    fprintf('\n========== Automatic judgement ==========\n');
    fprintf('Mean minimum obstacle distance: old %.3f, new %.3f\n', oldMin, newMin);
    fprintf('Mean average obstacle distance: old %.3f, new %.3f\n', oldAvg, newAvg);
    fprintf('Mean path length: old %.3f, new %.3f\n', oldLen, newLen);
    fprintf('Mean navigation time: old %.3f, new %.3f\n', oldTime, newTime);
    fprintf('Mean RMS angular-velocity change: old %.5f, new %.5f\n', oldRmsDw, newRmsDw);
    fprintf('Mean maximum heading change: old %.5f, new %.5f\n', oldMaxHeading, newMaxHeading);
    fprintf('Mean cumulative heading change: old %.5f, new %.5f\n', oldCumHeading, newCumHeading);

    if successNew && newRmsDw <= oldRmsDw && newMaxHeading <= oldMaxHeading && newLen <= oldLen * 1.10 && newTime <= oldTime * 1.10
        fprintf('Judgement: d_safe clearance is recommended. It reduces or maintains heading oscillation without obvious efficiency loss.\n');
    elseif successNew
        fprintf('Judgement: d_safe clearance remains feasible. Check angular-velocity and heading-change metrics before drawing conclusions.\n');
    else
        fprintf('Judgement: d_safe clearance caused failures in some cases. Consider tuning d_safe or sight_dist.\n');
    end
end

function rad = deg2rad_local(deg)
    rad = deg / 180 * pi;
end
