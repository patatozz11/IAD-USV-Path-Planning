function [Path, distanceX, OPEN_num, open, run_time] = Astar_improved_expand(GridMap, St, Ta, diagRule)
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

% remove start from OPEN (so we expand from it once)
OPEN(OPEN_COUNT,1) = 0;
open_index_map(xNode, yNode) = uint32(OPEN_COUNT);

% mark start as visited
closed_flag(xNode, yNode) = true;
NoPath = 1;

% ===================== Main loop =====================
while ((xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1)

    % 调用改进后的扩展函数 (传入 Target 坐标用于计算引导方向)
    exp_array = expand_array(xNode, yNode, path_cost, xTarget, yTarget, closed_flag, GridMap, MaxCol, MaxRow, diagRule, goal_distance);
    
    exp_count = size(exp_array,1);

    for i = 1:exp_count
        ex = exp_array(i,1);
        ey = exp_array(i,2);
        eg = exp_array(i,3);
        eh = exp_array(i,4);
        ef = exp_array(i,5);

        idx = open_index_map(ex, ey);

        if idx ~= 0
            % already discovered: relax if better f
            if ef <= OPEN(idx,8)
                OPEN(idx,4) = xNode;
                OPEN(idx,5) = yNode;
                OPEN(idx,6) = eg;
                OPEN(idx,7) = eh;
                OPEN(idx,8) = ef;
                OPEN(idx,1) = 1; % ensure it's considered OPEN
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
OPEN_num = size(OPEN, 1);
open = OPEN;

% -------------------- Reconstruct path --------------------
xval = xNode;
yval = yNode;

if (xval == xTarget) && (yval == yTarget)
    Target_ind = open_index_map(xval, yval); 
    
    if Target_ind == 0
        Path = []; distanceX = inf; return;
    end

    parent_x = OPEN(Target_ind,4);
    parent_y = OPEN(Target_ind,5);
    
    Optimal_path = zeros(MaxRow * MaxCol, 2);
    Optimal_path(1,:) = [xval, yval];
    path_len = 1;

    while (parent_x ~= xStart || parent_y ~= yStart)
        path_len = path_len + 1;
        Optimal_path(path_len, :) = [parent_x, parent_y];
        
        inode = open_index_map(parent_x, parent_y);
        
        if inode == 0
            Path = []; distanceX = inf; return;
        end
        
        parent_x = OPEN(inode,4);
        parent_y = OPEN(inode,5);
    end
    
    path_len = path_len + 1;
    Optimal_path(path_len, :) = [xStart, yStart];
    
    % 得到原始路径
    Path = flipud(Optimal_path(1:path_len, :));
    
    % --- 可选: Floyd 平滑 (如果您有这个函数请保留，没有则注释掉) ---
    Path = Floyd_Smooth_safer(Path, GridMap); 
    
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
%  改进后的扩展函数 
% =====================================================================
function exp_array = expand_array(node_x, node_y, gn, xTarget, yTarget, closed_flag, GridMap, MaxCol, MaxRow, diagRule, R)
    exp_array = zeros(0,5);
    exp_count = 0;
    
    % --- 1. 计算【当前节点 -> 目标节点】的方向引导 ---
    dx = xTarget - node_x;
    dy = yTarget - node_y;
    
    % 如果已经到达终点附近 (重合)，则不需要筛选方向
    if dx == 0 && dy == 0
        dir_x = 0; dir_y = 0;
    else
        % 按照论文 Table 1，将角度离散化到最近的 45度 (8个罗盘方向)
        angle_rad = atan2(dy, dx);
        discrete_angle = round(angle_rad / (pi/4)) * (pi/4);
        
        % 将离散后的角度转回单位向量 (-1, 0, 1)
        dir_x = round(cos(discrete_angle));
        dir_y = round(sin(discrete_angle));
    end
    % --------------------------------------------------------

    for k = 1:-1:-1
        for j = 1:-1:-1
            if (k==0 && j==0), continue; end
            
            % --- 2. 动态 5 邻域筛选 (Strict Implementation) ---
            % 基于向量点积：只保留目标方向前方 180度 内的节点
            if (dir_x ~= 0 || dir_y ~= 0)
                % 计算点积: Dot < 0 表示方向相反 (夹角 > 90度)
                dot_prod = dir_x * k + dir_y * j;
                if dot_prod < 0
                    continue; % 剔除背后的 3 个节点
                end
            end
            % -----------------------------------------------

            s_x = node_x + k;
            s_y = node_y + j;
            
            % 越界检查
            if (s_x <= 0 || s_x > MaxRow || s_y <= 0 || s_y > MaxCol)
                continue;
            end
            
            % 障碍物检查 (Closed Set 检查)
            if closed_flag(s_x, s_y)
                continue;
            end
            
            % 穿角规则检查
            if (k ~= 0 && j ~= 0) && (diagRule ~= 0)
                b1 = GridMap(node_x + k, node_y);
                b2 = GridMap(node_x, node_y + j);
                if diagRule == 1
                    if b1 || b2, continue; end
                elseif diagRule == 2
                    if b1 && b2, continue; end
                end
            end
            
            % 当前邻居到目标的距离 r
            h_val = euclidean_distance(xTarget, yTarget, s_x, s_y); 
            % 动态权重系数 (1 + r/R)
            weight = 1 + (h_val / R); 
            
            % 最终的评价函数 f
            % 将你原来的指数密度项与动态权重结合
            exp_count = exp_count + 1;
            exp_array(exp_count,1) = s_x;
            exp_array(exp_count,2) = s_y;
            exp_array(exp_count,3) = gn + euclidean_distance(node_x,node_y,s_x,s_y); % g
            exp_array(exp_count,4) = h_val; % h
            obsdensity = obs_density(s_x, s_y, xTarget, yTarget, GridMap, MaxRow, MaxCol);
            exp_array(exp_count,5) = exp_array(exp_count,3) + weight * (exp(obsdensity) * h_val);
        end
    end
end

% =====================================================================
%  障碍物密度计算函数
% =====================================================================
function P_obs = obs_density(node_x, node_y, xTarget, yTarget, GridMap, MaxRow, MaxCol)
    % 1. 确定当前节点到目标点构成的矩形区域
    x_min = min(node_x, xTarget);
    x_max = max(node_x, xTarget);
    y_min = min(node_y, yTarget);
    y_max = max(node_y, yTarget);
    
    % 2. 边界保护 (防止索引越界)
    x_min = max(1, x_min);
    x_max = min(MaxRow, x_max);
    y_min = max(1, y_min);
    y_max = min(MaxCol, y_max);
    
    % 3. 统计区域内的障碍物数量
    % GridMap 必须是 logical 或者数值型 (1=障碍)
    region = GridMap(x_min:x_max, y_min:y_max);
    node_obs = sum(region(:));
    
    % 4. 计算总格数
    total_grids = numel(region);
    
    % 5. 计算密度比率
    if total_grids > 0
        P_obs = node_obs / total_grids;
    else
        P_obs = 0;
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

% === min_fn ===
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