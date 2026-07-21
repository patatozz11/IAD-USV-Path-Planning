clc;
clear;
close all;

% Diagnose why the reported minimum obstacle distance is zero.
% This script does not modify DWA, LOS, IA*, or evaluation logic.

batchMode = true;
outDir = fullfile('results', 'min_distance_diagnosis');
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

%% Shared IA* + Floyd path
[Path, distanceX, OPEN_num, ~, run_time] = ...
    IAstar_FiveDir_Fallback_Fast(gridMap, start, goal, diagRule);
if isempty(Path)
    error('IA* failed to generate the shared global path.');
end

[obs_static_rows, obs_static_cols] = find(gridMap == 1);
obs_Static = [obs_static_rows, obs_static_cols];
obstacleCells = [obs_Static; Obs_Unknown];

%% Run noLOS baseline and recommended LOS
fprintf('\n===== Running noLOS baseline =====\n');
clear DWA_Fusion_noLOS
prepareBatchFigure(batchMode);
[Final_Path_noLOS, metrics_noLOS] = DWA_Fusion_noLOS( ...
    gridMap, obs_Static, Obs_Unknown, moveobs_trajectory, Path, start, goal, ...
    Kinematic, evalParam);
close all;

fprintf('\n===== Running recommended LOS =====\n');
losConfig.reach_dist = 1.0;
losConfig.sight_dist = 6.0;
losConfig.max_skip = 3;
d_safe = 0.2;
clear DWA_Fusion_LOS
prepareBatchFigure(batchMode);
[Final_Path_LOS, metrics_LOS] = DWA_Fusion_LOS( ...
    gridMap, obs_Static, Obs_Unknown, moveobs_trajectory, Path, start, goal, ...
    Kinematic, evalParam, d_safe, losConfig);
close all;

%% Independent distance diagnosis
diagNoLOS = diagnoseTrajectoryDistance(Final_Path_noLOS, obstacleCells);
diagLOS = diagnoseTrajectoryDistance(Final_Path_LOS, obstacleCells);

fprintf('\n========== Minimum Distance Diagnosis: noLOS ==========\n');
printDiagnosis(diagNoLOS);

fprintf('\n========== Minimum Distance Diagnosis: LOS ==========\n');
printDiagnosis(diagLOS);

fprintf('\nCoordinate convention check:\n');
fprintf('  Final_Path(:,1) is the grid-row/x coordinate used by obstacle rows.\n');
fprintf('  Final_Path(:,2) is the grid-column/y coordinate used by obstacle columns.\n');
fprintf('  Obstacles are drawn as cells [row,row+1] x [col,col+1].\n');
fprintf('  Path indices are converted to world coordinates by p_world = p_raw + 0.5 before distance calculation.\n');

[startBoundaryDist, startNearestCell, startCenterDist, startInside, startOnBoundary] = ...
    pointObstacleDiagnosis(start + 0.5, obstacleCells);
fprintf('\nStart-point coordinate check:\n');
fprintf('  Raw point: [%.3f, %.3f]\n', start(1), start(2));
fprintf('  World point: [%.3f, %.3f]\n', start(1)+0.5, start(2)+0.5);
fprintf('  Nearest obstacle cell: [%d, %d]\n', startNearestCell(1), startNearestCell(2));
fprintf('  Distance to obstacle boundary: %.6f\n', startBoundaryDist);
fprintf('  Distance to obstacle center: %.6f\n', startCenterDist);
fprintf('  Is inside obstacle cell: %d\n', startInside);
fprintf('  Is on obstacle boundary: %d\n', startOnBoundary);

save(fullfile(outDir, 'min_distance_diagnosis.mat'), ...
    'diagNoLOS', 'diagLOS', 'metrics_noLOS', 'metrics_LOS', ...
    'Final_Path_noLOS', 'Final_Path_LOS', 'Path', 'gridMap', ...
    'start', 'goal', 'obstacleCells', 'distanceX', 'OPEN_num', 'run_time');

plotDiagnosisFigure(gridMap, Path, Final_Path_noLOS, Final_Path_LOS, ...
    start, goal, diagNoLOS, diagLOS, outDir, false);
plotDiagnosisFigure(gridMap, Path, Final_Path_noLOS, Final_Path_LOS, ...
    start, goal, diagNoLOS, diagLOS, outDir, true);

fprintf('\nSaved diagnosis results to: %s\n', outDir);

%% Helper functions
function diagInfo = diagnoseTrajectoryDistance(traj, obstacleCells)
    n = size(traj, 1);
    distHistory = inf(n, 1);
    rawPointHistory = NaN(n, 2);
    worldPointHistory = NaN(n, 2);
    nearestCellHistory = zeros(n, 2);
    centerDistHistory = inf(n, 1);
    insideHistory = false(n, 1);
    boundaryHistory = false(n, 1);

    for ii = 1:n
        rawPoint = traj(ii, :);
        p = rawPoint + 0.5;
        rawPointHistory(ii, :) = rawPoint;
        worldPointHistory(ii, :) = p;
        [distHistory(ii), nearestCellHistory(ii,:), centerDistHistory(ii), ...
            insideHistory(ii), boundaryHistory(ii)] = pointObstacleDiagnosis(p, obstacleCells);
    end

    [minDistValue, minDistStep] = min(distHistory);
    minDistRawPoint = rawPointHistory(minDistStep, :);
    minDistWorldPoint = worldPointHistory(minDistStep, :);
    nearestObstacleCell = nearestCellHistory(minDistStep, :);
    distanceToObstacleCenter = centerDistHistory(minDistStep);
    isInsideObstacleCell = insideHistory(minDistStep);
    isOnObstacleBoundary = boundaryHistory(minDistStep);

    diagInfo.minDistValue = minDistValue;
    diagInfo.minDistStep = minDistStep;
    diagInfo.minDistPoint = minDistRawPoint;
    diagInfo.minDistRawPoint = minDistRawPoint;
    diagInfo.minDistWorldPoint = minDistWorldPoint;
    diagInfo.nearestObstacleCell = nearestObstacleCell;
    diagInfo.isInsideObstacleCell = isInsideObstacleCell;
    diagInfo.isOnObstacleBoundary = isOnObstacleBoundary;
    diagInfo.distanceToObstacleCenter = distanceToObstacleCenter;
    diagInfo.distanceToObstacleBoundary = minDistValue;
    diagInfo.distHistory = distHistory;
    diagInfo.rawPointHistory = rawPointHistory;
    diagInfo.worldPointHistory = worldPointHistory;
    diagInfo.insideCount = nnz(insideHistory);
    diagInfo.boundaryCount = nnz(boundaryHistory);
end

function [distBoundary, nearestCell, distCenter, inside, onBoundary] = pointObstacleDiagnosis(p, obstacleCells)
    tol = 1e-9;
    if isempty(obstacleCells)
        distBoundary = inf;
        nearestCell = [NaN, NaN];
        distCenter = inf;
        inside = false;
        onBoundary = false;
        return;
    end

    px = p(1);
    py = p(2);
    xMin = obstacleCells(:,1);
    xMax = obstacleCells(:,1) + 1;
    yMin = obstacleCells(:,2);
    yMax = obstacleCells(:,2) + 1;

    dx = max(max(xMin - px, 0), px - xMax);
    dy = max(max(yMin - py, 0), py - yMax);
    dists = sqrt(dx.^2 + dy.^2);
    [distBoundary, idx] = min(dists);
    nearestCell = obstacleCells(idx, :);

    centers = obstacleCells + 0.5;
    centerDists = hypot(centers(:,1) - px, centers(:,2) - py);
    distCenter = centerDists(idx);

    inX = px > nearestCell(1) + tol && px < nearestCell(1) + 1 - tol;
    inY = py > nearestCell(2) + tol && py < nearestCell(2) + 1 - tol;
    inside = inX && inY;

    withinXClosed = px >= nearestCell(1) - tol && px <= nearestCell(1) + 1 + tol;
    withinYClosed = py >= nearestCell(2) - tol && py <= nearestCell(2) + 1 + tol;
    onVerticalEdge = (abs(px - nearestCell(1)) <= tol || abs(px - nearestCell(1) - 1) <= tol) && withinYClosed;
    onHorizontalEdge = (abs(py - nearestCell(2)) <= tol || abs(py - nearestCell(2) - 1) <= tol) && withinXClosed;
    onBoundary = (onVerticalEdge || onHorizontalEdge) && ~inside;
end

function printDiagnosis(diagInfo)
    fprintf('Minimum distance value: %.6f\n', diagInfo.minDistValue);
    fprintf('Minimum distance step: %d\n', diagInfo.minDistStep);
    fprintf('Raw point: [%.6f, %.6f]\n', diagInfo.minDistRawPoint(1), diagInfo.minDistRawPoint(2));
    fprintf('World point: [%.6f, %.6f]\n', diagInfo.minDistWorldPoint(1), diagInfo.minDistWorldPoint(2));
    fprintf('Nearest obstacle cell: [%d, %d]\n', diagInfo.nearestObstacleCell(1), diagInfo.nearestObstacleCell(2));
    fprintf('Is inside obstacle cell: %d\n', diagInfo.isInsideObstacleCell);
    fprintf('Is on obstacle boundary: %d\n', diagInfo.isOnObstacleBoundary);
    fprintf('Distance to obstacle center: %.6f\n', diagInfo.distanceToObstacleCenter);
    fprintf('Distance to obstacle boundary: %.6f\n', diagInfo.distanceToObstacleBoundary);
    fprintf('Inside-obstacle trajectory samples: %d\n', diagInfo.insideCount);
    fprintf('Boundary-contact trajectory samples: %d\n', diagInfo.boundaryCount);
end

function plotDiagnosisFigure(gridMap, globalPath, pathNoLOS, pathLOS, start, goal, diagNoLOS, diagLOS, outDir, zoomMode)
    [maxRow, maxCol] = size(gridMap);
    figure('Visible', 'off', 'Color', 'w');
    hold on;
    axis equal;

    if zoomMode
        p = diagLOS.minDistWorldPoint;
        xlim([max(1, p(1)-3), min(maxRow+1, p(1)+3)]);
        ylim([max(1, p(2)-3), min(maxCol+1, p(2)+3)]);
    else
        axis([1 maxRow+1, 1 maxCol+1]);
    end

    set(gca, 'XTick', 1:maxRow+1, 'YTick', 1:maxCol+1, ...
        'XGrid', 'on', 'YGrid', 'on', 'GridLineStyle', '-', ...
        'XTickLabelRotation', 0);
    xlabel('Grid x');
    ylabel('Grid y');

    drawGridObstacles(gridMap);
    hGlobal = plot(globalPath(:,1)+0.5, globalPath(:,2)+0.5, 'b:', 'LineWidth', 2);
    hNoLOS = plot(pathNoLOS(:,1)+0.5, pathNoLOS(:,2)+0.5, 'r--', 'LineWidth', 2);
    hLOS = plot(pathLOS(:,1)+0.5, pathLOS(:,2)+0.5, 'b-', 'LineWidth', 2);
    hStart = plot(start(1)+0.5, start(2)+0.5, 'b^', 'LineWidth', 2, 'MarkerSize', 8);
    hGoal = plot(goal(1)+0.5, goal(2)+0.5, 'go', 'LineWidth', 2, 'MarkerSize', 8);

    % Mark raw minimum-distance points using the same +0.5 display offset as paths.
    hMinNoLOS = plot(diagNoLOS.minDistPoint(1)+0.5, diagNoLOS.minDistPoint(2)+0.5, ...
        'mo', 'MarkerSize', 10, 'LineWidth', 2);
    hMinLOS = plot(diagLOS.minDistPoint(1)+0.5, diagLOS.minDistPoint(2)+0.5, ...
        'ms', 'MarkerSize', 10, 'LineWidth', 2);
    drawObstacleBox(diagNoLOS.nearestObstacleCell, 'm', 2.0);
    drawObstacleBox(diagLOS.nearestObstacleCell, 'm', 2.0);

    text(start(1)+0.7, start(2)+0.5, 'Start', 'FontSize', 12);
    text(goal(1)+0.7, goal(2)+0.5, 'End', 'FontSize', 12);

    legend([hGlobal, hNoLOS, hLOS, hStart, hGoal, hMinNoLOS, hMinLOS], ...
        {'IA* + Floyd path', 'noLOS trajectory', 'LOS trajectory', ...
         'Start', 'End', 'noLOS min-distance point', 'LOS min-distance point'}, ...
        'Location', 'best');

    if zoomMode
        title('Minimum distance diagnosis (zoom)');
        saveas(gcf, fullfile(outDir, 'min_distance_diagnosis_zoom.fig'));
        print(gcf, fullfile(outDir, 'min_distance_diagnosis_zoom.png'), '-dpng', '-r600');
    else
        title('Minimum distance diagnosis');
        saveas(gcf, fullfile(outDir, 'min_distance_diagnosis.fig'));
        print(gcf, fullfile(outDir, 'min_distance_diagnosis.png'), '-dpng', '-r600');
    end
end

function drawGridObstacles(gridMap)
    [obsRows, obsCols] = find(gridMap == 1);
    for ii = 1:numel(obsRows)
        fill([obsRows(ii), obsRows(ii)+1, obsRows(ii)+1, obsRows(ii)], ...
             [obsCols(ii), obsCols(ii), obsCols(ii)+1, obsCols(ii)+1], ...
             'k', 'EdgeColor', 'none');
    end
end

function drawObstacleBox(cellXY, colorSpec, lineWidth)
    x = cellXY(1);
    y = cellXY(2);
    plot([x x+1 x+1 x x], [y y y+1 y+1 y], ...
        '-', 'Color', colorSpec, 'LineWidth', lineWidth);
end

function prepareBatchFigure(batchMode)
    if batchMode
        close all;
        figure('Visible', 'off', 'Color', 'w');
        hold on;
        axis equal;
    end
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
