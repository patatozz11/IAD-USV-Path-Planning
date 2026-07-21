clc;
clear;
close all;

projectRoot = fileparts(mfilename('fullpath'));
outDir = fullfile(projectRoot, 'results', 'fusion_los_overlay_comparison', 'search_30x30');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

raw20 = makeRaw20();
rowIdx = min(20, max(1, round(((1:30) - 0.5) * 20 / 30 + 0.5)));
colIdx = min(20, max(1, round(((1:30) - 0.5) * 20 / 30 + 0.5)));
baseRaw = raw20(rowIdx, colIdx);

starts = [3 9; 3 10; 4 10; 3 10; 3 10; 4 9];
goals  = [28 22; 28 22; 28 22; 27 22; 28 21; 27 22];
sightValues = [6.0, 7.5, 9.0, 10.0];
[variantNames, variantBlocks] = makeVariants();

global dt;
dt = 0.1;
Kinematic = [1.0, toRadian(20.0), 0.2, toRadian(50.0), 0.01, toRadian(1)];
evalParam = [0.05, 0.2, 0.1, 3.0];
d_safe = 0.2;
baseLosConfig = struct('reach_dist', 1.0, 'sight_dist', 9.0, 'max_skip', 3);

oldVisible = get(0, 'DefaultFigureVisible');
cleanupObj = onCleanup(@() set(0, 'DefaultFigureVisible', oldVisible)); %#ok<NASGU>
set(0, 'DefaultFigureVisible', 'off');

allResults = [];

%% Stage 1: all requested start/goal pairs on the strict nearest-neighbor map.
stage1 = [];
for k = 1:size(starts, 1)
    fprintf('\nSTAGE 1 %d/%d: start=[%d,%d], goal=[%d,%d], sight=9.0\n', ...
        k, size(starts, 1), starts(k, 1), starts(k, 2), goals(k, 1), goals(k, 2));
    r = runPair(baseRaw, starts(k, :), goals(k, :), 9.0, ...
        Kinematic, evalParam, d_safe, baseLosConfig);
    r.stage = "start_goal";
    r.variant = "nearest_neighbor_base";
    r.addedBlocks = zeros(0, 4);
    r = finishResult(r);
    stage1 = appendResult(stage1, r);
    allResults = appendResult(allResults, r);
    printResult(r);
end

bestStartIdx = chooseBest(stage1);
bestStart = stage1(bestStartIdx).start;
bestGoal = stage1(bestStartIdx).goal;

%% Stage 2: all requested LOS sight distances for the best start/goal pair.
stage2 = [];
for k = 1:numel(sightValues)
    if sightValues(k) == 9.0
        r = stage1(bestStartIdx);
        r.sightDist = 9.0;
    else
        fprintf('\nSTAGE 2 %d/%d: sight=%.1f\n', k, numel(sightValues), sightValues(k));
        r = runPair(baseRaw, bestStart, bestGoal, sightValues(k), ...
            Kinematic, evalParam, d_safe, baseLosConfig);
    end
    r.stage = "sight_distance";
    r.variant = "nearest_neighbor_base";
    r.addedBlocks = zeros(0, 4);
    r = finishResult(r);
    stage2 = appendResult(stage2, r);
    allResults = appendResult(allResults, r);
    printResult(r);
end

bestSightIdx = chooseBest(stage2);
bestSight = stage2(bestSightIdx).sightDist;

%% Stage 3: strict base plus four small, geometry-preserving map variants.
stage3 = [];
for k = 1:numel(variantNames)
    raw = applyBlocks(baseRaw, variantBlocks{k});
    if k == 1
        matching = find([stage2.sightDist] == bestSight, 1);
        r = stage2(matching);
    else
        fprintf('\nSTAGE 3 %d/%d: variant=%s\n', k, numel(variantNames), variantNames(k));
        r = runPair(raw, bestStart, bestGoal, bestSight, ...
            Kinematic, evalParam, d_safe, baseLosConfig);
    end
    r.stage = "map_variant";
    r.variant = variantNames(k);
    r.raw = raw;
    r.addedBlocks = variantBlocks{k};
    r = finishResult(r);
    stage3 = appendResult(stage3, r);
    allResults = appendResult(allResults, r);
    printResult(r);
end

bestMapIdx = chooseBest(stage3);
best = stage3(bestMapIdx);

T = resultsTable(allResults);
writetable(T, fullfile(outDir, 'candidate_LOS_30x30_summary.csv'));
save(fullfile(outDir, 'candidate_LOS_30x30_details.mat'), ...
    'allResults', 'stage1', 'stage2', 'stage3', 'raw20', 'baseRaw', ...
    'rowIdx', 'colIdx', 'variantNames', 'variantBlocks');
save(fullfile(outDir, 'best_LOS_30x30_candidate.mat'), 'best');

fprintf('\nBEST: variant=%s, start=[%d,%d], goal=[%d,%d], sight=%.1f\n', ...
    best.variant, best.start(1), best.start(2), best.goal(1), best.goal(2), best.sightDist);
fprintf('eligible=%d, criteria=%d, score=%.3f, nodes=%d\n', ...
    best.eligible, best.criteriaCount, best.score, best.globalNodes);
printResult(best);

function r = runPair(raw, start, goal, sightDist, Kinematic, evalParam, d_safe, baseLosConfig)
gridMap = rot90(raw, 3);
r = emptyResult();
r.raw = raw;
r.start = start;
r.goal = goal;
r.sightDist = sightDist;

if ~isFree(gridMap, start) || ~isFree(gridMap, goal)
    r.error = "Start or goal is not free under gridMap(x,y).";
    return;
end

[obsX, obsY] = find(gridMap == 1);
obsStatic = [obsX, obsY];
emptyObs = [];
try
    [globalPath, r.globalLength, ~, ~, ~] = ...
        IAstar_FiveDir_Fallback_Fast(gridMap, start, goal, 1);
    if isempty(globalPath)
        r.error = "Empty global path.";
        return;
    end
    r.globalNodes = size(globalPath, 1);
    r.globalTurns = max(0, size(globalPath, 1) - 2);

    clear DWA_Fusion_noLOS
    fig = figure('Visible', 'off');
    hold on;
    axis equal;
    [r.noPathData, noMetrics] = DWA_Fusion_noLOS( ...
        gridMap, obsStatic, emptyObs, emptyObs, globalPath, start, goal, ...
        Kinematic, evalParam);
    if isvalid(fig), close(fig); end

    losConfig = baseLosConfig;
    losConfig.sight_dist = sightDist;
    clear DWA_Fusion_LOS
    fig = figure('Visible', 'off');
    hold on;
    axis equal;
    [r.losPathData, losMetrics] = DWA_Fusion_LOS( ...
        gridMap, obsStatic, emptyObs, emptyObs, globalPath, start, goal, ...
        Kinematic, evalParam, d_safe, losConfig);
    if isvalid(fig), close(fig); end
    close(findall(0, 'Type', 'figure', 'Visible', 'off'));

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
    [r.meanTrajectoryDistance, r.maxTrajectoryDistance] = ...
        trajectoryDistance(r.noPathData(:, 1:2), r.losPathData(:, 1:2));
catch ME
    r.error = string(ME.message);
    close(findall(0, 'Type', 'figure', 'Visible', 'off'));
end
end

function r = finishResult(r)
r.criteriaCount = sum([r.losPath <= r.noPath + 0.05, ...
    r.losTime <= r.noTime + 0.5, ...
    r.losMin >= r.noMin || r.losMin >= 0.30, ...
    r.losAvg >= r.noAvg - 0.05, ...
    r.losAngular < r.noAngular]);
r.visibleDifference = r.maxTrajectoryDistance >= 0.10;
r.eligible = r.noSuccess && r.losSuccess && ...
    r.losPath <= r.noPath + 0.30 && r.losTime <= r.noTime + 1.0 && ...
    r.losMin >= 0.30 && r.losAvg >= r.noAvg - 0.08 && ...
    r.losAngular < r.noAngular && r.visibleDifference && ...
    r.globalNodes >= 5;
r.score = scoreResult(r);
end

function score = scoreResult(r)
if ~(r.noSuccess && r.losSuccess)
    score = -inf;
    return;
end
score = 100 * r.criteriaCount + 35 * (r.noAngular - r.losAngular) ...
    + 10 * (r.noPath - r.losPath) + 4 * (r.noTime - r.losTime) ...
    + 8 * (r.losMin - r.noMin) + 5 * (r.losAvg - r.noAvg) ...
    + 5 * min(r.globalNodes, 8) + 5 * min(r.maxTrajectoryDistance, 1.5);
if r.losMin < 0.30
    score = score - 250 * (0.30 - r.losMin);
end
if r.globalNodes < 5
    score = score - 100 * (5 - r.globalNodes);
end
if r.eligible
    score = score + 500;
end
end

function idx = chooseBest(results)
eligible = find([results.eligible]);
if ~isempty(eligible)
    [~, ii] = max([results(eligible).score]);
    idx = eligible(ii);
else
    successful = find([results.noSuccess] & [results.losSuccess]);
    [~, ii] = max([results(successful).score]);
    idx = successful(ii);
end
end

function a = appendResult(a, r)
if isempty(a)
    a = r;
else
    a(end + 1) = r;
end
end

function r = emptyResult()
r = struct('stage', "", 'variant', "", 'addedBlocks', zeros(0, 4), ...
    'raw', [], 'start', [NaN NaN], 'goal', [NaN NaN], 'sightDist', NaN, ...
    'globalLength', NaN, 'globalNodes', 0, 'globalTurns', 0, ...
    'noSuccess', false, 'losSuccess', false, 'noPath', NaN, 'losPath', NaN, ...
    'noTime', NaN, 'losTime', NaN, 'noMin', NaN, 'losMin', NaN, ...
    'noAvg', NaN, 'losAvg', NaN, 'noAngular', NaN, 'losAngular', NaN, ...
    'meanTrajectoryDistance', inf, 'maxTrajectoryDistance', inf, ...
    'criteriaCount', 0, 'visibleDifference', false, 'eligible', false, ...
    'score', -inf, 'noPathData', [], 'losPathData', [], 'error', "");
end

function printResult(r)
fprintf(['  success=%d/%d nodes=%d | path %.3f/%.3f | time %.1f/%.1f | ', ...
    'min %.3f/%.3f | avg %.3f/%.3f | angular %.3f/%.3f | diff %.3f/%.3f | eligible=%d\n'], ...
    r.noSuccess, r.losSuccess, r.globalNodes, r.noPath, r.losPath, ...
    r.noTime, r.losTime, r.noMin, r.losMin, r.noAvg, r.losAvg, ...
    r.noAngular, r.losAngular, r.meanTrajectoryDistance, ...
    r.maxTrajectoryDistance, r.eligible);
end

function T = resultsTable(r)
T = table(string({r.stage})', string({r.variant})', vertcat(r.start), ...
    vertcat(r.goal), [r.sightDist]', [r.noSuccess]', [r.losSuccess]', ...
    [r.noPath]', [r.losPath]', [r.noTime]', [r.losTime]', ...
    [r.noMin]', [r.losMin]', [r.noAvg]', [r.losAvg]', ...
    [r.noAngular]', [r.losAngular]', [r.globalNodes]', ...
    [r.meanTrajectoryDistance]', [r.maxTrajectoryDistance]', ...
    [r.criteriaCount]', [r.eligible]', [r.score]', ...
    'VariableNames', {'Stage', 'Variant', 'Start', 'Goal', 'SightDistance', ...
    'NoLOSSuccess', 'LOSSuccess', 'NoLOSPath', 'LOSPath', 'NoLOSTime', ...
    'LOSTime', 'NoLOSMin', 'LOSMin', 'NoLOSAvg', 'LOSAvg', ...
    'NoLOSAngular', 'LOSAngular', 'GlobalNodes', ...
    'MeanTrajectoryDistance', 'MaxTrajectoryDistance', ...
    'CriteriaCount', 'Eligible', 'Score'});
end

function [names, blocks] = makeVariants()
names = ["nearest_neighbor_base"; "bank_notches"; "bridge_rear"; ...
    "midwater_sparse"; "mixed_local"];
blocks = cell(5, 1);
blocks{1} = zeros(0, 4);
blocks{2} = [8 8 16 17; 22 22 21 22];
blocks{3} = [15 15 13 14; 18 18 20 20];
blocks{4} = [14 14 21 21; 17 17 22 23; 12 12 25 25];
blocks{5} = [8 8 16 17; 17 17 20 21; 21 22 25 25];
end

function raw = applyBlocks(raw, blocks)
for k = 1:size(blocks, 1)
    raw(blocks(k, 1):blocks(k, 2), blocks(k, 3):blocks(k, 4)) = 1;
end
end

function tf = isFree(gridMap, p)
tf = p(1) >= 1 && p(1) <= size(gridMap, 1) && ...
    p(2) >= 1 && p(2) <= size(gridMap, 2) && gridMap(p(1), p(2)) == 0;
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
