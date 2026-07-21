function [Path, distanceX, OPEN_num, open, run_time, debugInfo] = IAstar_FiveDir_Fallback(GridMap, St, Ta, diagRule)
% Final IA* global search with optimized five-direction expansion and lazy fallback.
%
% The algorithm uses the goal-oriented five-direction expansion as the
% default search mode. When the current node has no valid five-direction
% candidate, or has only one valid five-direction candidate while no other
% active OPEN branch is available, it temporarily restores eight-neighborhood
% expansion only for this local node. The next iteration returns to
% five-direction expansion.
% The fallback mechanism is a local robustness compensation for constrained
% regions and does not relax obstacle checking, diagonal anti-corner-cutting,
% obstacle density weighting, or the A* cost function.
%
% Inputs:
%   GridMap        : 0=free, 1=obstacle
%   St, Ta         : [x y] start/target
%   diagRule       : 0 allow corner cutting (default)
%                    1 forbid diagonal if EITHER adjacent orthogonal cell is blocked
%                    2 forbid diagonal only if BOTH adjacent orthogonal cells are blocked
%
% Outputs:
%   Path, distanceX, OPEN_num, open, run_time(ms), debugInfo
%   run_time only measures the main A* search loop. It excludes
%   initialization, path reconstruction, Floyd smoothing, plotting,
%   printing, and file saving.

if nargin < 4 || isempty(diagRule)
    diagRule = 0;
end

totalTimer = tic;
debugInfo = struct();

% -------------------- Init --------------------
[MaxRow, MaxCol] = size(GridMap);
xStart  = St(1);  yStart  = St(2);
xTarget = Ta(1);  yTarget = Ta(2);

GridMap = logical(GridMap);
closed_flag = GridMap; % static obstacles + expanded nodes

% OPEN table: [flag, x, y, parent_x, parent_y, g, h, f]
OPEN = zeros(MaxRow * MaxCol, 8);
OPEN_COUNT = 0;
open_index_map = zeros(MaxRow, MaxCol, 'uint32');

xNode = xStart;
yNode = yStart;
path_cost = 0;
goal_distance = euclidean_distance(xNode, yNode, xTarget, yTarget);

OPEN_COUNT = OPEN_COUNT + 1;
OPEN(OPEN_COUNT,:) = insert_open(xNode, yNode, xNode, yNode, path_cost, goal_distance, goal_distance);
OPEN(OPEN_COUNT,1) = 0;
open_index_map(xNode, yNode) = uint32(OPEN_COUNT);
closed_flag(xNode, yNode) = true;

NoPath = 1;

fallbackTriggeredTimes = 0;
fallbackExpandedNodes = 0;
totalExpandedNodes = 0;

% ===================== Main loop =====================
planningTimer = tic;
while ((xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1)
    totalExpandedNodes = totalExpandedNodes + 1;

    [exp_array, five_count] = expand_array_fivedir_fast( ...
        xNode, yNode, path_cost, xTarget, yTarget, closed_flag, GridMap, ...
        MaxCol, MaxRow, diagRule, goal_distance);

    % Lazy fallback: temporarily restore the full eight-neighborhood
    % expansion only when the local five-direction expansion is blocked, or
    % when it leaves only one branch and there is no other active OPEN
    % alternative. This avoids entering a one-way dead end in highly
    % constrained narrow channels.
    fallbackNeeded = (five_count == 0);
    if ~fallbackNeeded && five_count <= 1
        fallbackNeeded = (active_open_count(OPEN, OPEN_COUNT) == 0);
    end

    if fallbackNeeded
        fallbackTriggeredTimes = fallbackTriggeredTimes + 1;
        [exp_array, eight_count] = expand_array_eightdir_fallback( ...
            xNode, yNode, path_cost, xTarget, yTarget, closed_flag, GridMap, ...
            MaxCol, MaxRow, diagRule, goal_distance);
        if eight_count > 0
            fallbackExpandedNodes = fallbackExpandedNodes + 1;
        end
    end

    exp_count = size(exp_array,1);
    for i = 1:exp_count
        ex = exp_array(i,1);
        ey = exp_array(i,2);
        eg = exp_array(i,3);
        eh = exp_array(i,4);
        ef = exp_array(i,5);

        idx = open_index_map(ex, ey);
        if idx ~= 0
            if ef <= OPEN(idx,8)
                OPEN(idx,4) = xNode;
                OPEN(idx,5) = yNode;
                OPEN(idx,6) = eg;
                OPEN(idx,7) = eh;
                OPEN(idx,8) = ef;
                OPEN(idx,1) = 1;
            end
        else
            OPEN_COUNT = OPEN_COUNT + 1;
            OPEN(OPEN_COUNT,:) = insert_open(ex, ey, xNode, yNode, eg, eh, ef);
            open_index_map(ex, ey) = uint32(OPEN_COUNT);
        end
    end

    index_min_node = min_fn(OPEN, OPEN_COUNT);
    if index_min_node ~= -1
        xNode = OPEN(index_min_node,2);
        yNode = OPEN(index_min_node,3);
        path_cost = OPEN(index_min_node,6);

        closed_flag(xNode, yNode) = true;
        OPEN(index_min_node,1) = 0;
    else
        NoPath = 0;
    end
end

run_time = toc(planningTimer) * 1000;
OPEN = OPEN(1:OPEN_COUNT,:);
OPEN_num = size(OPEN, 1);
open = OPEN;

% -------------------- Reconstruct path --------------------
if xStart == xTarget && yStart == yTarget
    Path = [xStart, yStart];
    distanceX = 0;
else
    [Path, distanceX] = reconstruct_path(xNode, yNode, xStart, yStart, xTarget, yTarget, ...
        OPEN, open_index_map, MaxRow, MaxCol, GridMap);
end

if nargout >= 6
    debugInfo = build_debug_info(run_time, toc(totalTimer) * 1000, ...
        fallbackTriggeredTimes, fallbackExpandedNodes, totalExpandedNodes);
end
end

function [exp_array, five_count] = expand_array_fivedir_fast(node_x, node_y, gn, xTarget, yTarget, closed_flag, GridMap, MaxCol, MaxRow, diagRule, R)
    candidate_dirs = five_direction_lookup(xTarget - node_x, yTarget - node_y);
    exp_array = zeros(size(candidate_dirs, 1), 5);
    five_count = 0;

    for dirIdx = 1:size(candidate_dirs, 1)
        k = candidate_dirs(dirIdx, 1);
        j = candidate_dirs(dirIdx, 2);
        s_x = node_x + k;
        s_y = node_y + j;

        if (s_x <= 0 || s_x > MaxRow || s_y <= 0 || s_y > MaxCol)
            continue;
        end
        if closed_flag(s_x, s_y)
            continue;
        end

        if (k ~= 0 && j ~= 0) && (diagRule ~= 0)
            b1 = GridMap(node_x + k, node_y);
            b2 = GridMap(node_x, node_y + j);
            if diagRule == 1
                if b1 || b2
                    continue;
                end
            elseif diagRule == 2
                if b1 && b2
                    continue;
                end
            end
        end

        five_count = five_count + 1;
        exp_array(five_count,:) = make_candidate(node_x, node_y, s_x, s_y, gn, ...
            xTarget, yTarget, GridMap, MaxRow, MaxCol, R);
    end

    exp_array = exp_array(1:five_count,:);
end

function dirs = five_direction_lookup(dx, dy)
    dir_index = target_direction_index(dx, dy);
    switch dir_index
        case 1 % [ 1,  1]
            dirs = [1 1; 1 0; 1 -1; 0 1; -1 1];
        case 2 % [ 1,  0]
            dirs = [1 1; 1 0; 1 -1; 0 1; 0 -1];
        case 3 % [ 1, -1]
            dirs = [1 1; 1 0; 1 -1; 0 -1; -1 -1];
        case 4 % [ 0,  1]
            dirs = [1 1; 1 0; 0 1; -1 1; -1 0];
        case 5 % [ 0, -1]
            dirs = [1 0; 1 -1; 0 -1; -1 0; -1 -1];
        case 6 % [-1,  1]
            dirs = [1 1; 0 1; -1 1; -1 0; -1 -1];
        case 7 % [-1,  0]
            dirs = [0 1; 0 -1; -1 1; -1 0; -1 -1];
        case 8 % [-1, -1]
            dirs = [1 -1; 0 -1; -1 1; -1 0; -1 -1];
        otherwise
            dirs = [1 1; 1 0; 1 -1; 0 1; 0 -1; -1 1; -1 0; -1 -1];
    end
end

function idx = target_direction_index(dx, dy)
    if dx == 0 && dy == 0
        idx = 0;
        return;
    end

    sx = sign(dx);
    sy = sign(dy);
    ax = abs(dx);
    ay = abs(dy);
    tan22_5 = 0.4142135623730951;
    tan67_5 = 2.4142135623730950;

    if ay <= ax * tan22_5
        dir_x = sx;
        dir_y = 0;
    elseif ay >= ax * tan67_5
        dir_x = 0;
        dir_y = sy;
    else
        dir_x = sx;
        dir_y = sy;
    end

    if dir_x == 1 && dir_y == 1
        idx = 1;
    elseif dir_x == 1 && dir_y == 0
        idx = 2;
    elseif dir_x == 1 && dir_y == -1
        idx = 3;
    elseif dir_x == 0 && dir_y == 1
        idx = 4;
    elseif dir_x == 0 && dir_y == -1
        idx = 5;
    elseif dir_x == -1 && dir_y == 1
        idx = 6;
    elseif dir_x == -1 && dir_y == 0
        idx = 7;
    else
        idx = 8;
    end
end

function [exp_array, eight_count] = expand_array_eightdir_fallback(node_x, node_y, gn, xTarget, yTarget, closed_flag, GridMap, MaxCol, MaxRow, diagRule, R)
    exp_array = zeros(8,5);
    eight_count = 0;

    for k = 1:-1:-1
        for j = 1:-1:-1
            if (k == 0 && j == 0)
                continue;
            end

            s_x = node_x + k;
            s_y = node_y + j;
            if (s_x <= 0 || s_x > MaxRow || s_y <= 0 || s_y > MaxCol)
                continue;
            end
            if closed_flag(s_x, s_y)
                continue;
            end

            if (k ~= 0 && j ~= 0) && (diagRule ~= 0)
                b1 = GridMap(node_x + k, node_y);
                b2 = GridMap(node_x, node_y + j);
                if diagRule == 1
                    if b1 || b2
                        continue;
                    end
                elseif diagRule == 2
                    if b1 && b2
                        continue;
                    end
                end
            end

            eight_count = eight_count + 1;
            exp_array(eight_count,:) = make_candidate(node_x, node_y, s_x, s_y, gn, ...
                xTarget, yTarget, GridMap, MaxRow, MaxCol, R);
        end
    end

    exp_array = exp_array(1:eight_count,:);
end

function candidate = make_candidate(node_x, node_y, s_x, s_y, gn, xTarget, yTarget, GridMap, MaxRow, MaxCol, R)
    h_val = euclidean_distance(xTarget, yTarget, s_x, s_y);
    weight = 1 + (h_val / R);
    obsdensity = obs_density(s_x, s_y, xTarget, yTarget, GridMap, MaxRow, MaxCol);

    candidate = zeros(1,5);
    candidate(1) = s_x;
    candidate(2) = s_y;
    candidate(3) = gn + euclidean_distance(node_x,node_y,s_x,s_y);
    candidate(4) = h_val;
    candidate(5) = candidate(3) + weight * (exp(obsdensity) * h_val);
end

function [Path, distanceX] = reconstruct_path(xNode, yNode, xStart, yStart, xTarget, yTarget, OPEN, open_index_map, MaxRow, MaxCol, GridMap)
    if ~(xNode == xTarget && yNode == yTarget)
        Path = [];
        distanceX = inf;
        return;
    end

    Target_ind = open_index_map(xNode, yNode);
    if Target_ind == 0
        Path = [];
        distanceX = inf;
        return;
    end

    parent_x = OPEN(Target_ind,4);
    parent_y = OPEN(Target_ind,5);
    Optimal_path = zeros(MaxRow * MaxCol, 2);
    Optimal_path(1,:) = [xNode, yNode];
    path_len = 1;

    while (parent_x ~= xStart || parent_y ~= yStart)
        path_len = path_len + 1;
        Optimal_path(path_len, :) = [parent_x, parent_y];

        inode = open_index_map(parent_x, parent_y);
        if inode == 0
            Path = [];
            distanceX = inf;
            return;
        end

        parent_x = OPEN(inode,4);
        parent_y = OPEN(inode,5);
    end

    path_len = path_len + 1;
    Optimal_path(path_len, :) = [xStart, yStart];
    Path = flipud(Optimal_path(1:path_len, :));
    Path = Floyd_Smooth_safer(Path, GridMap);

    distanceX = 0;
    for ii = 1:(size(Path,1)-1)
        distanceX = distanceX + euclidean_distance(Path(ii,1),Path(ii,2), Path(ii+1,1),Path(ii+1,2));
    end
end

function P_obs = obs_density(node_x, node_y, xTarget, yTarget, GridMap, MaxRow, MaxCol)
    x_min = min(node_x, xTarget);
    x_max = max(node_x, xTarget);
    y_min = min(node_y, yTarget);
    y_max = max(node_y, yTarget);

    x_min = max(1, x_min);
    x_max = min(MaxRow, x_max);
    y_min = max(1, y_min);
    y_max = min(MaxCol, y_max);

    region = GridMap(x_min:x_max, y_min:y_max);
    total_grids = numel(region);
    if total_grids > 0
        P_obs = sum(region(:)) / total_grids;
    else
        P_obs = 0;
    end
end

function dist = euclidean_distance(x1, y1, x2, y2)
    dist = sqrt((x2 - x1)^2 + (y2 - y1)^2);
end

function new_row = insert_open(xval, yval, parent_xval, parent_yval, gn, hn, fn)
    new_row = [1, xval, yval, parent_xval, parent_yval, gn, hn, fn];
end

function i_min = min_fn(OPEN, OPEN_COUNT)
    best_f = inf;
    i_min = -1;
    for j = 1:OPEN_COUNT
        if OPEN(j,1) == 1
            if OPEN(j,8) < best_f
                best_f = OPEN(j,8);
                i_min = j;
            end
        end
    end
end

function count = active_open_count(OPEN, OPEN_COUNT)
    count = 0;
    for j = 1:OPEN_COUNT
        if OPEN(j,1) == 1
            count = count + 1;
        end
    end
end

function debugInfo = build_debug_info(purePlanningTime, totalRuntime, fallbackTriggeredTimes, fallbackExpandedNodes, totalExpandedNodes)
    debugInfo.fallbackTriggeredTimes = fallbackTriggeredTimes;
    debugInfo.fallbackExpandedNodes = fallbackExpandedNodes;
    debugInfo.totalExpandedNodes = totalExpandedNodes;
    debugInfo.fallbackExpandedRatio = fallbackExpandedNodes / max(totalExpandedNodes, 1);
    debugInfo.whetherFallbackActuallyUsed = fallbackExpandedNodes > 0;
    debugInfo.purePlanningTime = purePlanningTime;
    debugInfo.totalRuntime = totalRuntime;
end
