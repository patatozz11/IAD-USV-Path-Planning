function [minObsDist, avgObsDist, distList, samplePts] = calc_path_obstacle_distance(Path, gridMap, sampleStep)
%CALC_PATH_OBSTACLE_DISTANCE Clearance metrics for a grid path trajectory.
%   Each obstacle cell is treated as a unit square centered at its grid index.

if nargin < 3 || isempty(sampleStep)
    sampleStep = 0.05;
end

if sampleStep <= 0
    error('sampleStep must be positive.');
end

if isempty(Path)
    minObsDist = NaN;
    avgObsDist = NaN;
    distList = [];
    samplePts = [];
    return;
end

samplePts = sample_path_segments(Path, sampleStep);

[obsX, obsY] = find(gridMap == 1);
numSamples = size(samplePts, 1);

if isempty(obsX)
    distList = inf(numSamples, 1);
    minObsDist = Inf;
    avgObsDist = Inf;
    return;
end

distList = zeros(numSamples, 1);
for i = 1:numSamples
    distList(i) = min_distance_to_obstacle_cells(samplePts(i, :), obsX, obsY);
end

minObsDist = min(distList);
avgObsDist = mean(distList);
end

function samplePts = sample_path_segments(Path, sampleStep)
samplePts = Path(1, :);

for i = 1:size(Path, 1) - 1
    p0 = Path(i, :);
    p1 = Path(i + 1, :);
    segment = p1 - p0;
    segmentLength = hypot(segment(1), segment(2));

    if segmentLength == 0
        continue;
    end

    sampleDistances = (sampleStep:sampleStep:segmentLength)';
    t = sampleDistances / segmentLength;
    t = t(t < 1);

    if ~isempty(t)
        samplePts = [samplePts; p0 + t .* segment]; %#ok<AGROW>
    end

    samplePts = [samplePts; p1]; %#ok<AGROW>
end
end

function minDist = min_distance_to_obstacle_cells(point, obsX, obsY)
px = point(1);
py = point(2);

xmin = obsX - 0.5;
xmax = obsX + 0.5;
ymin = obsY - 0.5;
ymax = obsY + 0.5;

dx = max(max(xmin - px, 0), px - xmax);
dy = max(max(ymin - py, 0), py - ymax);

minDist = min(hypot(dx, dy));
end
