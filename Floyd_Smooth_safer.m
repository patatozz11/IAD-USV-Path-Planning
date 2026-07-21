function [new_path, stats] = Floyd_Smooth_safer(path, map, d_safe)
% Floyd path smoothing with optional clearance-aware shortcut rejection.
%
% Usage:
%   new_path = Floyd_Smooth_safer(path, map)
%   new_path = Floyd_Smooth_safer(path, map, d_safe)
%   [new_path, stats] = Floyd_Smooth_safer(...)
%
% d_safe = 0 keeps the original behavior: a shortcut is accepted when the
% line segment does not pass through obstacle grids according to the
% supercover DDA check. d_safe > 0 additionally rejects a shortcut if the
% segment clearance to obstacle-grid boundaries is smaller than d_safe.

    if nargin < 3 || isempty(d_safe)
        d_safe = 0.25;
    end

    map = logical(map);
    originalNodeCount = size(path, 1);

    stats = init_stats(d_safe, originalNodeCount);
    if isempty(path)
        new_path = path;
        stats.success = false;
        return;
    end

    obstacleCells = obstacle_cell_list(map);
    new_path = remove_colinear_points(path);

    i = 1;
    while i < size(new_path, 1) - 1
        best = i + 1;
        rejected_here = 0;
        for j = i + 2 : size(new_path, 1)
            [ok, clearance, rejectionReason] = is_segment_safe_with_clearance( ...
                new_path(i,:), new_path(j,:), map, d_safe, obstacleCells);
            stats.shortcutChecks = stats.shortcutChecks + 1;
            if isfinite(clearance)
                stats.minShortcutClearance = min(stats.minShortcutClearance, clearance);
            end

            if ok
                best = j;
            elseif strcmp(rejectionReason, 'clearance')
                rejected_here = rejected_here + 1;
            end
        end

        stats.rejectedShortcuts = stats.rejectedShortcuts + rejected_here;
        if best > i + 1
            stats.removedNodes = stats.removedNodes + (best - i - 1);
            new_path(i+1:best-1,:) = [];
        end
        i = i + 1;
    end

    stats.finalNodeCount = size(new_path, 1);
    stats.pathLength = path_length(new_path);
    stats.turningNodes = count_turning_nodes(new_path);
    stats.minClearance = path_min_clearance(new_path, obstacleCells);
    stats.success = ~isempty(new_path) && all_path_nodes_free(new_path, map);
end

function stats = init_stats(d_safe, originalNodeCount)
    stats = struct();
    stats.d_safe = d_safe;
    stats.success = true;
    stats.originalNodeCount = originalNodeCount;
    stats.finalNodeCount = 0;
    stats.removedNodes = 0;
    stats.shortcutChecks = 0;
    stats.rejectedShortcuts = 0;
    stats.pathLength = inf;
    stats.turningNodes = 0;
    stats.minClearance = inf;
    stats.minShortcutClearance = inf;
end

function out = remove_colinear_points(p)
    n = size(p, 1);
    if n < 3
        out = p;
        return;
    end

    d = diff(p, 1, 1);
    turn = d(1:end-1,1) .* d(2:end,2) - d(1:end-1,2) .* d(2:end,1);
    keep = [true; turn ~= 0; true];
    out = p(keep,:);
end

function [ok, clearance, rejectionReason] = is_segment_safe_with_clearance(a, b, map, d_safe, obstacleCells)
    rejectionReason = '';
    if ~is_line_passable(a, b, map)
        ok = false;
        clearance = 0;
        rejectionReason = 'collision';
        return;
    end

    if d_safe <= 0
        ok = true;
        clearance = segment_min_clearance(a, b, obstacleCells);
        return;
    end

    clearance = segment_min_clearance(a, b, obstacleCells);
    ok = clearance >= d_safe;
    if ~ok
        rejectionReason = 'clearance';
    end
end

function ok = is_line_passable(a, b, map)
    [R, C] = size(map);

    x1 = a(1) + 0.5;  y1 = a(2) + 0.5;
    x2 = b(1) + 0.5;  y2 = b(2) + 0.5;

    ix = floor(x1);  iy = floor(y1);
    ex = floor(x2);  ey = floor(y2);

    if ix < 1 || ix > R || iy < 1 || iy > C || ex < 1 || ex > R || ey < 1 || ey > C
        ok = false;
        return;
    end
    if map(ix, iy) || map(ex, ey)
        ok = false;
        return;
    end

    dx = x2 - x1;  dy = y2 - y1;
    sx = sign(dx); sy = sign(dy);

    if sx == 0
        tMaxX = inf; tDeltaX = inf;
    else
        nextVX = ix + (sx > 0);
        tMaxX = (nextVX - x1) / dx;
        tDeltaX = 1 / abs(dx);
    end

    if sy == 0
        tMaxY = inf; tDeltaY = inf;
    else
        nextHY = iy + (sy > 0);
        tMaxY = (nextHY - y1) / dy;
        tDeltaY = 1 / abs(dy);
    end

    while ix ~= ex || iy ~= ey
        if tMaxX < tMaxY
            ix = ix + sx;
            if ix < 1 || ix > R || map(ix, iy)
                ok = false;
                return;
            end
            tMaxX = tMaxX + tDeltaX;
        elseif tMaxY < tMaxX
            iy = iy + sy;
            if iy < 1 || iy > C || map(ix, iy)
                ok = false;
                return;
            end
            tMaxY = tMaxY + tDeltaY;
        else
            nx = ix + sx; ny = iy + sy;
            if nx < 1 || nx > R || ny < 1 || ny > C || map(nx, iy) || map(ix, ny) || map(nx, ny)
                ok = false;
                return;
            end

            ix = nx; iy = ny;
            tMaxX = tMaxX + tDeltaX;
            tMaxY = tMaxY + tDeltaY;
        end
    end

    ok = true;
end

function obstacleCells = obstacle_cell_list(map)
    [obsX, obsY] = find(map);
    obstacleCells = [obsX, obsY];
end

function minClearance = segment_min_clearance(a, b, obstacleCells)
    if isempty(obstacleCells)
        minClearance = inf;
        return;
    end

    p1 = [a(1) + 0.5, a(2) + 0.5];
    p2 = [b(1) + 0.5, b(2) + 0.5];
    segLen = norm(p2 - p1);
    sampleStep = 0.1;
    sampleCount = max(2, ceil(segLen / sampleStep) + 1);

    minClearance = inf;
    for idx = 1:sampleCount
        t = (idx - 1) / (sampleCount - 1);
        p = p1 + t * (p2 - p1);
        minClearance = min(minClearance, point_to_obstacle_boundary_distance(p, obstacleCells));
        if minClearance == 0
            return;
        end
    end
end

function minDist = point_to_obstacle_boundary_distance(p, obstacleCells)
    px = p(1);
    py = p(2);
    xMin = obstacleCells(:,1);
    xMax = obstacleCells(:,1) + 1;
    yMin = obstacleCells(:,2);
    yMax = obstacleCells(:,2) + 1;

    dx = max(max(xMin - px, 0), px - xMax);
    dy = max(max(yMin - py, 0), py - yMax);
    dist = sqrt(dx.^2 + dy.^2);
    minDist = min(dist);
end

function minClearance = path_min_clearance(path, obstacleCells)
    if isempty(path)
        minClearance = inf;
        return;
    end

    minClearance = inf;
    if size(path, 1) == 1
        minClearance = segment_min_clearance(path(1,:), path(1,:), obstacleCells);
        return;
    end

    for ii = 1:size(path, 1)-1
        minClearance = min(minClearance, segment_min_clearance(path(ii,:), path(ii+1,:), obstacleCells));
    end
end

function total = path_length(path)
    total = 0;
    for ii = 1:size(path, 1)-1
        total = total + hypot(path(ii+1,1) - path(ii,1), path(ii+1,2) - path(ii,2));
    end
end

function count = count_turning_nodes(path)
    if size(path, 1) < 3
        count = 0;
        return;
    end

    count = 0;
    for ii = 2:size(path, 1)-1
        v1 = path(ii,:) - path(ii-1,:);
        v2 = path(ii+1,:) - path(ii,:);
        if v1(1) * v2(2) - v1(2) * v2(1) ~= 0
            count = count + 1;
        end
    end
end

function free = all_path_nodes_free(path, map)
    [R, C] = size(map);
    free = true;
    for ii = 1:size(path, 1)
        x = path(ii, 1);
        y = path(ii, 2);
        if x < 1 || x > R || y < 1 || y > C || map(x, y)
            free = false;
            return;
        end
    end
end
