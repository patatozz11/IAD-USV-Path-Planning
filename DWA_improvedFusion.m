function traj = DWA_improvedFusion(gridMap, obs_Static, Obs_Unknown, moveobs_trajectory, Path, start, goal, Kinematic, evalParam)
    %% 1. 初始化参数
    global dt;     
    obstacleR = 0.5; % 障碍物膨胀半径
    
    % 初始化状态 x=[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
    if size(Path,1) > 1
        init_theta = atan2(Path(2,2)-Path(1,2), Path(2,1)-Path(1,1));
    else
        init_theta = atan2(goal(2)-start(2), goal(1)-start(1));
    end
    x = [start(1), start(2), init_theta, 0, 0]';
    
    traj = x(1:2)'; 
    obsMove_num = size(moveobs_trajectory, 1);
    
    %% 2. 绘图准备
    h_robot = plot(x(1)+0.5, x(2)+0.5, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 6);
    h_arrow = quiver(x(1)+0.5, x(2)+0.5, 0.5*cos(x(3)), 0.5*sin(x(3)), 'b', 'LineWidth', 2);
    h_dyn_fill = fill([0 0 0 0], [0 0 0 0], 'y', 'EdgeColor', 'k');
    %h_trail = plot(x(1)+0.5, x(2)+0.5, 'b.', 'MarkerSize', 2);
    % 改成红色实线 (r-)，线宽 2，这样实际跑出来的路径就是一条红色的粗线
    h_trail = plot(x(1)+0.5, x(2)+0.5, 'r-', 'LineWidth', 1);
    
    h_green_lines = [];
    % 初始化"当前目标点"的红色星号句柄
    h_current_target = plot(NaN, NaN, 'r*', 'MarkerSize', 10, 'LineWidth', 1.5);
    
    %% === 【数据记录】 ===
    hist_time = [];
    hist_v = [];
    hist_w = [];
    hist_theta = [];
    
    disp('DWA (Improved Strategy) 仿真开始...');
    tic;
    
    %% 3. 仿真主循环
    MaxSteps = 5000;
    
    for i = 1:MaxSteps
        % --- A. 获取动态障碍物 ---
        curr_dyn_obs = [];
        if obsMove_num > 0
            idx = min(i, obsMove_num);
            curr_dyn_obs = moveobs_trajectory(idx, :);
            cx = curr_dyn_obs(1); cy = curr_dyn_obs(2);
            set(h_dyn_fill, 'XData', [cx, cx+1, cx+1, cx], 'YData', [cy, cy, cy+1, cy+1]);
        end
        
        % --- B. 障碍物融合 ---
        obstacle = [obs_Static; Obs_Unknown; curr_dyn_obs];
        
        % --- C. 【核心修改】寻找局部目标 (使用论文改进策略) ---

        % 传入 obstacle (包含静态+动态实时信息)
        local_goal = LocalGoal_Strategy(x, Path, goal, obstacle);
        set(h_current_target, 'XData', local_goal(1)+0.5, 'YData', local_goal(2)+0.5);
        
        % --- D. DWA 核心计算 ---
        [u, traj_candidates] = DynamicWindowApproach(x, Kinematic, local_goal, evalParam, obstacle, obstacleR);
        
        % --- E. 运动更新 ---
        x = f(x, u);
        traj = [traj; x(1:2)'];
        
        % === 【记录】当前状态 ===
        hist_time = [hist_time; i * dt];
        hist_v = [hist_v; x(4)];
        hist_w = [hist_w; x(5)];
        hist_theta = [hist_theta; x(3)];
        
        % --- F. 绘图更新 ---
        delete(h_green_lines); 
        h_green_lines = [];
        if ~isempty(traj_candidates)
            for it = 1 : size(traj_candidates, 1)/5
                ind = 1 + (it-1)*5;
                px = traj_candidates(ind, :) + 0.5;   
                py = traj_candidates(ind+1, :) + 0.5; 
                h_line = plot(px, py, '-g', 'LineWidth', 1.5);
                h_green_lines = [h_green_lines; h_line]; 
            end
        end
        set(h_robot, 'XData', x(1)+0.5, 'YData', x(2)+0.5);
        set(h_arrow, 'XData', x(1)+0.5, 'YData', x(2)+0.5, 'UData', cos(x(3)), 'VData', sin(x(3)));
        set(h_trail, 'XData', traj(:,1)+0.5, 'YData', traj(:,2)+0.5);
        
        drawnow limitrate;
        
        % --- G. 判断到达 ---
        dist_to_goal = sqrt((x(1)-goal(1))^2 + (x(2)-goal(2))^2);
        if dist_to_goal < 0.5
            real_navigation_time = i * dt;
            % 计算路径长度 
            % diff(traj) 计算相邻两点的差值 (dx, dy)
            % .^2 平方 -> sum(..., 2) 行求和 -> sqrt 开根号 -> 得到每一步的距离
            % sum(...) 把所有步的距离加起来 -> 得到总长度
            path_length = sum(sqrt(sum(diff(traj).^2, 2)));
            disp(['到达终点!']);
            disp(['实际航行时间: ', num2str(real_navigation_time), ' s']); 
            disp(['实际路径长度: ', num2str(path_length), ' m']);  % <--- 显示长度
            disp(['程序计算耗时: ', num2str(toc), ' s']);
            
            % 画出三张漂亮的曲线图
            PlotMotionCurves(hist_time, hist_v, hist_w, hist_theta);
            break;
        end
    end 
end


%%   子函数区域：包含改进的目标选择策略、视线检测、绘图

function lg = LocalGoal_Strategy(x, path, final_goal, obstacles)
    persistent k;
    
    % --- 1. 初始化保护 ---
    % 如果是第一次运行，或者 k 还没初始化，或者 k 越界了，重置为 1
    if isempty(k) || k > size(path,1)
        k = 1; 
    end
    
    pos = x(1:2)'; 
    n = size(path,1);
    
    % --- 2. 异常情况处理 ---
    if n == 0, lg = final_goal; return; end
    
    % 如果已经离终点非常近 (0.5m)，直接锁定终点
    if norm(pos - final_goal) < 0.5
        lg = final_goal;
        return;
    end
    
    % --- 3. 确保 k 不越界 ---
    k = min(k, n);
    
    % 如果已经到达最后一个路径点，就一直返回终点，不再做切换判断
    if k == n
        lg = path(n, :);
        return;
    end
    
    % --- 4. 核心切换逻辑 (修复了“偷跑”BUG) ---
    current_target = path(k, :);
    next_target    = path(k+1, :);
    
    dist_to_current = norm(pos - current_target);
    
    % 定义两个关键距离：
    % reach_dist: "由于惯性可能略过点"的容差距离 (必须切)
    % sight_dist: "允许视线优化的最大距离" (只有船在这个范围内，才允许看视线切点)
    reach_dist = 1.0; 
    sight_dist = 3.0; 
    
    % 逻辑分支 A: 已经贴脸了 (距离 < 1.0m)，必须切换到下一个
    if dist_to_current < reach_dist
        k = k + 1;
        
    % 逻辑分支 B: 还没贴脸，但在预瞄范围内 (距离 < 3.0m)，且下个点视线无遮挡
    % 只有满足 "dist < sight_dist"，才允许执行视线检测！
    % 这就是防止一开始就把所有点都切完的“锁”！
    elseif dist_to_current < sight_dist
        safe_width = 1.0; % 视线检测宽度
        if isLineSafe(pos, next_target, obstacles, safe_width)
            k = k + 1;
        end
    end
    
    % --- 5. 输出 ---
    % 再次防止 k+1 后越界
    k = min(k, n);
    lg = path(k, :);
end

% ... (下方的 isLineSafe 等子函数不需要动) ...

%% 2. 几何视线检测函数 (通用性最强)
% 判断线段 p1-p2 是否安全 (即：没有任何一个障碍物离线段的距离 < safe_dist)
function safe = isLineSafe(p1, p2, obstacles, safe_dist)
    safe = true;
    if isempty(obstacles), return; end
    
    % 线段向量
    v_line = p2 - p1;
    len_sq = sum(v_line.^2);
    
    % 防止重合点报错
    if len_sq < 1e-6, return; end
    
    % --- 向量化计算所有障碍物到直线的距离 ---
    
    % 1. 计算障碍物点到 p1 的向量
    % obstacles 是 Nx2 矩阵
    v_obs = obstacles - p1; 
    
    % 2. 计算投影比例 t = (v_obs · v_line) / |v_line|^2
    % t 表示障碍物在线段上的投影位置，0~1 表示在线段中间
    t = (v_obs * v_line') / len_sq;
    
    % 3. 限制 t 在 [0, 1] 之间 (夹紧到线段范围内)
    t = max(0, min(1, t));
    
    % 4. 找到线段上距离障碍物最近的点 closest_point
    closest_points = p1 + t .* v_line;
    
    % 5. 计算障碍物到最近点的距离
    dists = sqrt(sum((obstacles - closest_points).^2, 2));
    
    % 6. 只要有一个障碍物的距离小于安全距离，就不安全
    if any(dists < safe_dist)
        safe = false;
    end
end

%% 3. Bresenham 直线栅格化算法
function [x, y] = bresenhamGrid(x1, y1, x2, y2)
    dx = abs(x2 - x1);
    dy = abs(y2 - y1);
    sx = sign(x2 - x1);
    sy = sign(y2 - y1);
    
    x = x1; y = y1;
    x_list = x; y_list = y;
    
    if dx > dy
        err = dx / 2;
        while x ~= x2
            x = x + sx;
            err = err - dy;
            if err < 0
                y = y + sy;
                err = err + dx;
            end
            x_list(end+1) = x; %#ok<AGROW>
            y_list(end+1) = y; %#ok<AGROW>
        end
    else
        err = dy / 2;
        while y ~= y2
            y = y + sy;
            err = err - dx;
            if err < 0
                x = x + sx;
                err = err + dy;
            end
            x_list(end+1) = x; %#ok<AGROW>
            y_list(end+1) = y; %#ok<AGROW>
        end
    end
    x = x_list; y = y_list;
end

%% 4. 绘图辅助函数 (XY轴留白 + 终点红点)
function PlotMotionCurves(t, v, w, theta)
    % 准备通用参数：X轴最大值增加 10% 的余量
    max_t = max(t);
    if max_t == 0, max_t = 1; end
    x_limit = [0, max_t * 1.1]; 
    
    % --- 线速度图 ---
    figure('Name', 'Linear Velocity', 'Color', 'w', 'NumberTitle', 'off'); 
    plot(t, v, 'b-', 'LineWidth', 2.0); hold on;
    plot(t(end), v(end), 'r.', 'MarkerSize', 15); 
    title('(a) Linear Velocity', 'FontSize', 12);
    xlabel('Time (s)', 'FontSize', 12); ylabel('Velocity (m/s)', 'FontSize', 12);
    grid on; xlim(x_limit);
    max_v = max(v); if max_v==0, max_v=1; end; ylim([0, max_v*1.2]);
    set(gca, 'FontSize', 10, 'LineWidth', 1.0);

    % --- 角速度图 ---
    figure('Name', 'Angular Velocity', 'Color', 'w', 'NumberTitle', 'off'); 
    plot(t, w, 'r-', 'LineWidth', 2.0); hold on;
    plot(t(end), w(end), 'b.', 'MarkerSize', 15);
    title('(b) Angular Velocity', 'FontSize', 12);
    xlabel('Time (s)', 'FontSize', 12); ylabel('Angular Vel (rad/s)', 'FontSize', 12);
    grid on; xlim(x_limit);
    max_val_w = max(abs(w)); if max_val_w==0, max_val_w=0.1; end
    ylim([-max_val_w*1.2, max_val_w*1.2]);
    set(gca, 'FontSize', 10, 'LineWidth', 1.0);

    % --- 航向角图 ---
    figure('Name', 'Heading Attitude', 'Color', 'w', 'NumberTitle', 'off'); 
    plot(t, theta, 'k-', 'LineWidth', 2.0); hold on;
    plot(t(end), theta(end), 'r.', 'MarkerSize', 15);
    title('(c) Heading Attitude', 'FontSize', 12);
    xlabel('Time (s)', 'FontSize', 12); ylabel('Heading (rad)', 'FontSize', 12);
    grid on; xlim(x_limit);
    min_h = min(theta); max_h = max(theta); range_h = max_h - min_h;
    if range_h==0, range_h=1; end
    ylim([min_h - range_h*0.1, max_h + range_h*0.1]);
    set(gca, 'FontSize', 10, 'LineWidth', 1.0);
end