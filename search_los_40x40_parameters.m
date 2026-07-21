clc;
clear;
close all;

projectRoot = fileparts(mfilename('fullpath'));
outDir = fullfile(projectRoot, 'results', 'fusion_los_overlay_comparison', 'search_40x40');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

gridMap_raw_20 = makeRaw20();
gridMap_raw_40 = kron(gridMap_raw_20, ones(2));
gridMap = rot90(gridMap_raw_40, 3);
start = [4, 14];
goal = [38, 30];

algorithmStartFree = gridMap(start(1), start(2)) == 0;
algorithmGoalFree = gridMap(goal(1), goal(2)) == 0;
literalYXStartFree = gridMap(start(2), start(1)) == 0;
literalYXGoalFree = gridMap(goal(2), goal(1)) == 0;
fprintf('Algorithm gridMap(x,y) check: start=%d, goal=%d\n', ...
    algorithmStartFree, algorithmGoalFree);
fprintf('Literal gridMap(y,x) check:   start=%d, goal=%d\n', ...
    literalYXStartFree, literalYXGoalFree);
assert(algorithmStartFree, 'Start point is on an obstacle under the project coordinate convention.');
assert(algorithmGoalFree, 'Goal point is on an obstacle under the project coordinate convention.');

global dt;
dt = 0.1;
Kinematic = [1.0, toRadian(20.0), 0.2, toRadian(50.0), 0.01, toRadian(1)];
evalParam = [0.05, 0.2, 0.1, 3.0];

paramNames = ["A_strict_double"; "B_smaller_sight"; ...
    "C_larger_sight"; "D_original_dsafe"];
sightValues = [12.0; 10.0; 14.0; 12.0];
reachValues = [2.0; 2.0; 2.0; 2.0];
dSafeValues = [0.4; 0.4; 0.4; 0.2];

[obsX, obsY] = find(gridMap == 1);
obsStatic = [obsX, obsY];
emptyObs = [];
[globalPath, globalLength, openNum, ~, planningTime] = ...
    IAstar_FiveDir_Fallback_Fast(gridMap, start, goal, 1);
assert(~isempty(globalPath), 'IA* failed to find a global path.');
globalNodes = size(globalPath, 1);

oldVisible = get(0, 'DefaultFigureVisible');
cleanupObj = onCleanup(@() set(0, 'DefaultFigureVisible', oldVisible)); %#ok<NASGU>
set(0, 'DefaultFigureVisible', 'off');

fprintf('\nRunning shared Without LOS trajectory...\n');
clear DWA_Fusion_noLOS
fig = figure('Visible', 'off');
hold on;
axis equal;
[noPathData, noMetrics] = DWA_Fusion_noLOS(gridMap, obsStatic, ...
    emptyObs, emptyObs, globalPath, start, goal, Kinematic, evalParam);
if isvalid(fig), close(fig); end

results = repmat(emptyResult(), numel(paramNames), 1);
for k = 1:numel(paramNames)
    fprintf('\nPARAMETER %s: sight=%.1f reach=%.1f d_safe=%.1f\n', ...
        paramNames(k), sightValues(k), reachValues(k), dSafeValues(k));
    losConfig = struct('reach_dist', reachValues(k), ...
        'sight_dist', sightValues(k), 'max_skip', 3);
    clear DWA_Fusion_LOS
    fig = figure('Visible', 'off');
    hold on;
    axis equal;
    [losPathData, losMetrics] = DWA_Fusion_LOS(gridMap, obsStatic, ...
        emptyObs, emptyObs, globalPath, start, goal, Kinematic, evalParam, ...
        dSafeValues(k), losConfig);
    if isvalid(fig), close(fig); end
    close(findall(0, 'Type', 'figure', 'Visible', 'off'));

    r = emptyResult();
    r.name = paramNames(k);
    r.sightDist = sightValues(k);
    r.reachDist = reachValues(k);
    r.dSafe = dSafeValues(k);
    r.noSuccess = logical(noMetrics.success);
    r.losSuccess = logical(losMetrics.success);
    r.noPath = noMetrics.path_length;
    r.losPath = losMetrics.path_length;
    r.noTime = noMetrics.navigation_time;
    r.losTime = losMetrics.navigation_time;
    r.noMin = noMetrics.minimum_obstacle_distance;
    r.losMin = losMetrics.minimum_obstacle_distance;
    r.noAvg = noMetrics.average_obstacle_distance;
    r.losAvg = losMetrics.average_obstacle_distance;
    r.noAngular = noMetrics.cumulative_delta_angular_velocity;
    r.losAngular = losMetrics.cumulative_delta_angular_velocity;
    r.globalNodes = globalNodes;
    r.noPathData = noPathData;
    r.losPathData = losPathData;
    [r.meanTrajectoryDistance, r.maxTrajectoryDistance] = ...
        trajectoryDistance(noPathData(:, 1:2), losPathData(:, 1:2));
    r.criteriaCount = sum([r.losPath <= r.noPath + 0.10, ...
        r.losTime <= r.noTime + 0.5, r.losMin >= r.noMin - 0.05, ...
        r.losAvg >= r.noAvg - 0.05, r.losAngular < r.noAngular]);
    r.eligible = r.noSuccess && r.losSuccess && ...
        r.losPath <= r.noPath + 0.30 && r.losTime <= r.noTime + 1.0 && ...
        r.losMin >= r.noMin - 0.05 && r.losAvg >= r.noAvg - 0.08 && ...
        r.losAngular < r.noAngular && r.maxTrajectoryDistance >= 0.10 && ...
        r.globalNodes >= 5;
    r.score = scoreResult(r);
    results(k) = r;
    fprintf(['  success=%d/%d nodes=%d | path %.3f/%.3f | time %.1f/%.1f | ', ...
        'min %.3f/%.3f | avg %.3f/%.3f | angular %.3f/%.3f | diff %.3f/%.3f | eligible=%d\n'], ...
        r.noSuccess, r.losSuccess, r.globalNodes, r.noPath, r.losPath, ...
        r.noTime, r.losTime, r.noMin, r.losMin, r.noAvg, r.losAvg, ...
        r.noAngular, r.losAngular, r.meanTrajectoryDistance, ...
        r.maxTrajectoryDistance, r.eligible);
end

eligibleIds = find([results.eligible]);
if ~isempty(eligibleIds)
    [~, ii] = max([results(eligibleIds).score]);
    bestIdx = eligibleIds(ii);
else
    [~, bestIdx] = max([results.score]);
end
best = results(bestIdx);

T = resultsTable(results);
writetable(T, fullfile(outDir, 'candidate_LOS_40x40_parameter_summary.csv'));
writematrix(gridMap_raw_40, fullfile(outDir, 'strict_kron_gridMap_raw_40.csv'));
save(fullfile(outDir, 'candidate_LOS_40x40_details.mat'), ...
    'results', 'best', 'T', 'gridMap_raw_20', 'gridMap_raw_40', ...
    'gridMap', 'start', 'goal', 'globalPath', 'globalLength', ...
    'globalNodes', 'openNum', 'planningTime', 'Kinematic', 'evalParam', ...
    'algorithmStartFree', 'algorithmGoalFree', ...
    'literalYXStartFree', 'literalYXGoalFree');

fprintf('\nBEST: %s, sight=%.1f, reach=%.1f, d_safe=%.1f, eligible=%d\n', ...
    best.name, best.sightDist, best.reachDist, best.dSafe, best.eligible);
disp(T);

function score = scoreResult(r)
if ~(r.noSuccess && r.losSuccess)
    score = -inf;
    return;
end
score = 100 * r.criteriaCount + 40 * (r.noAngular - r.losAngular) ...
    + 10 * (r.noPath - r.losPath) + 5 * (r.noTime - r.losTime) ...
    + 8 * (r.losMin - r.noMin) + 5 * (r.losAvg - r.noAvg) ...
    + 4 * min(r.maxTrajectoryDistance, 2.0);
if r.eligible
    score = score + 500;
end
end

function r = emptyResult()
r = struct('name', "", 'sightDist', NaN, 'reachDist', NaN, 'dSafe', NaN, ...
    'noSuccess', false, 'losSuccess', false, 'noPath', NaN, 'losPath', NaN, ...
    'noTime', NaN, 'losTime', NaN, 'noMin', NaN, 'losMin', NaN, ...
    'noAvg', NaN, 'losAvg', NaN, 'noAngular', NaN, 'losAngular', NaN, ...
    'globalNodes', 0, 'meanTrajectoryDistance', inf, ...
    'maxTrajectoryDistance', inf, 'criteriaCount', 0, 'eligible', false, ...
    'score', -inf, 'noPathData', [], 'losPathData', []);
end

function T = resultsTable(r)
T = table(string({r.name})', [r.sightDist]', [r.reachDist]', [r.dSafe]', ...
    [r.noSuccess]', [r.losSuccess]', [r.noPath]', [r.losPath]', ...
    [r.noTime]', [r.losTime]', [r.noMin]', [r.losMin]', ...
    [r.noAvg]', [r.losAvg]', [r.noAngular]', [r.losAngular]', ...
    [r.globalNodes]', [r.meanTrajectoryDistance]', ...
    [r.maxTrajectoryDistance]', [r.criteriaCount]', [r.eligible]', [r.score]', ...
    'VariableNames', {'ParameterSet', 'SightDistance', 'ReachDistance', ...
    'DSafe', 'NoLOSSuccess', 'LOSSuccess', 'NoLOSPath', 'LOSPath', ...
    'NoLOSTime', 'LOSTime', 'NoLOSMin', 'LOSMin', 'NoLOSAvg', 'LOSAvg', ...
    'NoLOSAngular', 'LOSAngular', 'GlobalNodes', ...
    'MeanTrajectoryDistance', 'MaxTrajectoryDistance', ...
    'CriteriaCount', 'Eligible', 'Score'});
end

function [meanDist, maxDist] = trajectoryDistance(a, b)
a = resamplePath(a, 201);
b = resamplePath(b, 201);
d = hypot(a(:, 1) - b(:, 1), a(:, 2) - b(:, 2));
meanDist = mean(d);
maxDist = max(d);
end

function sampled = resamplePath(path, count)
s = [0; cumsum(hypot(diff(path(:, 1)), diff(path(:, 2))))];
[s, idx] = unique(s, 'stable');
path = path(idx, :);
q = linspace(0, s(end), count)';
sampled = [interp1(s, path(:, 1), q), interp1(s, path(:, 2), q)];
end

function raw = makeRaw20()
raw = [
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 1 1 0 0 0 0 0 0 0 1 1 0 0 0 0;
    0 0 0 0 0 1 1 0 0 0 0 0 0 0 1 1 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0;
    0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 1 0 0 0;
    0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 1 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1
];
end

function radian = toRadian(degree)
radian = degree / 180 * pi;
end
