function [Path, distanceX, OPEN_num, open, run_time] = Astar_improved(GridMap, St, Ta, diagRule)
% Inputs:
%   GridMap : 0=free, 1=obstacle
%   St, Ta  : [x y] start/target
%   diagRule: 0 allow corner cutting (default)
%             1 forbid diagonal if EITHER adjacent orth cell is blocked
%             2 forbid diagonal only if BOTH adjacent orth cells are blocked
%
% Outputs:
%   Path, distanceX, OPEN_num, open, run_time(ms)

if nargin < 4 || isempty(diagRule)
    diagRule = 0;
end

t0 = tic;
% -------------------- Init --------------------
[MaxRow, MaxCol] = size(GridMap);

xStart  = St(1);  yStart  = St(2);
xTarget = Ta(1);  yTarget = Ta(2);

GridMap = logical(GridMap);

% closed_flag includes static obstacles + visited nodes
closed_flag = GridMap;

% OPEN table: [flag, x, y, parent_x, parent_y, g, h, f]
% flag=1 in OPEN, flag=0 removed/expanded
OPEN = zeros(MaxRow * MaxCol, 8);
OPEN_COUNT = 0;

% fast map: open_index_map(x,y) = row index in OPEN (0 means not present)
open_index_map = zeros(MaxRow, MaxCol, 'uint32');

% start node
xNode = xStart;
yNode = yStart;
path_cost = 0;
goal_distance = euclidean_distance(xNode, yNode, xTarget, yTarget);

OPEN_COUNT = OPEN_COUNT + 1;
OPEN(OPEN_COUNT,:) = insert_open(xNode, yNode, xNode, yNode, path_cost, goal_distance, goal_distance);

% remove start from OPEN (so we expand from it once, like your original)
OPEN(OPEN_COUNT,1) = 0;
open_index_map(xNode, yNode) = uint32(OPEN_COUNT);

% mark start as visited
closed_flag(xNode, yNode) = true;
NoPath = 1;

% ===================== Main loop =====================
while ((xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1)

    exp_array = expand_array(xNode, yNode, path_cost, xTarget, yTarget, closed_flag, GridMap, MaxCol, MaxRow, diagRule);
    exp_count = size(exp_array,1);

    for i = 1:exp_count
        ex = exp_array(i,1);
        ey = exp_array(i,2);
        eg = exp_array(i,3);
        eh = exp_array(i,4);
        ef = exp_array(i,5);

        idx = open_index_map(ex, ey);

        if idx ~= 0
            % already discovered: relax if better f (or you can compare g; your original used f)
            if ef <= OPEN(idx,8)
                OPEN(idx,4) = xNode;
                OPEN(idx,5) = yNode;
                OPEN(idx,6) = eg;
                OPEN(idx,7) = eh;
                OPEN(idx,8) = ef;
                OPEN(idx,1) = 1; % ensure it's considered OPEN (normally already 1)
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

        closed_flag(xNode, yNode) = true; % now visited
        OPEN(index_min_node,1) = 0;       % remove from OPEN
    else
        NoPath = 0;
    end
end

run_time = toc(t0) * 1000;

% Trim OPEN to actual size for output
OPEN = OPEN(1:OPEN_COUNT,:);

% current OPEN set count (flag==1)
OPEN_num = size(OPEN, 1);
open = OPEN;

% -------------------- Reconstruct path --------------------
xval = xNode;
yval = yNode;

if (xval == xTarget) && (yval == yTarget)
    % 1. 直接用 open_index_map 查终点索引 (O(1) 速度)
    Target_ind = open_index_map(xval, yval); 
    
    if Target_ind == 0
        Path = []; distanceX = inf; return;
    end

    parent_x = OPEN(Target_ind,4);
    parent_y = OPEN(Target_ind,5);
    
    % 预分配内存 (优化: 假设路径最长不超过地图格子数，最后再截断)
    Optimal_path = zeros(MaxRow * MaxCol, 2);
    Optimal_path(1,:) = [xval, yval];
    path_len = 1;

    while (parent_x ~= xStart || parent_y ~= yStart)
        path_len = path_len + 1;
        Optimal_path(path_len, :) = [parent_x, parent_y];
        
        % 2. 直接用 open_index_map 查父节点索引 (O(1) 速度)
        inode = open_index_map(parent_x, parent_y);
        
        if inode == 0
            % 异常保护
            Path = []; distanceX = inf; return;
        end
        
        parent_x = OPEN(inode,4);
        parent_y = OPEN(inode,5);
    end
    
    path_len = path_len + 1;
    Optimal_path(path_len, :) = [xStart, yStart];
    
    % 截断多余的预分配空间并反转
    Path = flipud(Optimal_path(1:path_len, :));

    % Floyd优化路径
    Path = Floyd_Smooth_safer(Path, GridMap);
    %Path = Floyd_Double_Pass(Path,GridMap);
    
    % 计算距离
    distanceX = 0;
    for ii = 1:(size(Path,1)-1)
        distanceX = distanceX + euclidean_distance(Path(ii,1),Path(ii,2), Path(ii+1,1),Path(ii+1,2));
    end
else
    Path = [];
    distanceX = inf;
end
end

% =====================================================================
function exp_array = expand_array(node_x, node_y, gn, xTarget, yTarget, closed_flag, GridMap, MaxCol, MaxRow, diagRule)
exp_array = zeros(0,5);
exp_count = 0;

for k = 1:-1:-1
    for j = 1:-1:-1
        if (k==0 && j==0), continue; end

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
                if b1 || b2, continue; end
            elseif diagRule == 2
                if b1 && b2, continue; end
            end
        end

        exp_count = exp_count + 1;
        exp_array(exp_count,1) = s_x;
        exp_array(exp_count,2) = s_y;
        exp_array(exp_count,3) = gn + euclidean_distance(node_x,node_y,s_x,s_y); % g
        exp_array(exp_count,4) = euclidean_distance(xTarget,yTarget,s_x,s_y);     % h
        obsdensity = obs_density(s_x, s_y, xTarget, yTarget, GridMap, MaxRow, MaxCol);       %考虑障碍物密度信息
        exp_array(exp_count,5) = exp_array(exp_count,3) + exp(obsdensity) * exp_array(exp_count,4); %f
    end
end
end

% === euclidean_distance ===
function dist = euclidean_distance(x1, y1, x2, y2)
dist = sqrt((x2 - x1)^2 + (y2 - y1)^2);
end

% === insert_open ===
function new_row = insert_open(xval, yval, parent_xval, parent_yval, gn, hn, fn)
new_row = [1, xval, yval, parent_xval, parent_yval, gn, hn, fn];
end

% === min_fn: GLOBAL min f over all OPEN(flag==1), no early break ===
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

function P_obs = obs_density(node_x, node_y, xTarget, yTarget, GridMap, MaxRow, MaxCol)
    % Inputs:
    %   node_x, node_y: 当前节点的整数坐标
    %   closed_flag: 静态障碍物矩阵 (1=障碍, 0=空)
    
    % 1. 确定最小外接矩形边界 (直接利用整数坐标，不需要 floor/ceil 也不要 +/- 0.8)
    x_min = min(node_x, xTarget);
    x_max = max(node_x, xTarget);
    y_min = min(node_y, yTarget);
    y_max = max(node_y, yTarget);
    
    % 2. 边界安全检查 (防止索引越界，虽然理论上不应该越界)
    x_min = max(1, x_min);
    x_max = min(MaxRow, x_max);
    y_min = max(1, y_min);
    y_max = min(MaxCol, y_max);
    
    % 3. 计算区域内的障碍物总数 (利用矩阵切片求和)
    node_obs = sum(GridMap(x_min:x_max, y_min:y_max), 'all');

    % 4. 计算区域总格数 (论文中的 Nx * Ny)
    % 注意: 比如从2到2，距离是0，但包含1个格子，所以要 +1
    N_x = (x_max - x_min) + 1;
    N_y = (y_max - y_min) + 1;
    total_grids = N_x * N_y;
    
    % 5. 计算密度
    if total_grids > 0
        P_obs = node_obs / total_grids;
    else
        P_obs = 0;
    end
end