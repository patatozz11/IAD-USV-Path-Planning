function traj = DWA_Fusion(~, obs_Static, Obs_Unknown, moveobs_trajectory, Path, start, goal, Kinematic, evalParam)
    %% 1. 初始化参数
    global dt;     
    obstacleR = 0.5; 
    
    % 初始化状态 x=[x, y, yaw, v, w]
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
    h_current_target = plot(NaN, NaN, 'r*', 'MarkerSize', 10, 'LineWidth', 1.5);
    
    %% === 【新增】数据记录数组 ===
    % 用于存储每一时刻的 [时间, 线速度, 角速度, 航向角]
    hist_time = [];
    hist_v = [];
    hist_w = [];
    hist_theta = [];
    
    %% === 初始化路径跟踪索引 ===
    path_idx = 2; 
    reach_dist = 1; 
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
        
        % --- C. 寻找局部目标 ---
        current_target = Path(min(path_idx, size(Path, 1)), :);
        dist_to_target = norm(x(1:2)' - current_target);
        
        if dist_to_target < reach_dist && path_idx < size(Path, 1)
            path_idx = path_idx + 1; 
            current_target = Path(path_idx, :); 
        end
        local_goal = current_target;
        set(h_current_target, 'XData', local_goal(1)+0.5, 'YData', local_goal(2)+0.5);
        
        % --- D. DWA 核心计算 ---
        [u, traj_candidates] = DynamicWindowApproach(x, Kinematic, local_goal, evalParam, obstacle, obstacleR);
        
        % --- E. 运动更新 ---
        x = f(x, u);
        traj = [traj; x(1:2)'];
        
        % === 【新增】记录当前状态数据 ===
        hist_time = [hist_time; i * dt];     % 记录当前时间
        hist_v = [hist_v; x(4)];             % 记录线速度 v
        hist_w = [hist_w; x(5)];             % 记录角速度 w
        hist_theta = [hist_theta; x(3)];     % 记录航向角 theta
        
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
        
        % --- G. 判断到达终点 ---
        dist_to_final_goal = sqrt((x(1)-goal(1))^2 + (x(2)-goal(2))^2);
        if dist_to_final_goal < 0.5
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
            
            % === 【新增】到达终点后，弹出三个新的波形图 ===
            PlotMotionCurves(hist_time, hist_v, hist_w, hist_theta);
            
            break;
        end
    end 
end



%% === 绘图辅助函数 ===
function PlotMotionCurves(t, v, w, theta)
    % 准备通用参数：X轴最大值增加 10% 的余量
    max_t = max(t);
    if max_t == 0, max_t = 1; end
    x_limit = [0, max_t * 1.1]; % 让X轴多显示 10% 的长度
    
    % ---------------------------------------------------------
    % 1. 线速度图 (Linear Velocity)
    % ---------------------------------------------------------
    figure('Name', 'Linear Velocity', 'Color', 'w', 'NumberTitle', 'off'); 
    plot(t, v, 'b-', 'LineWidth', 2.0); 
    hold on;
    % 在终点画个红点，方便看清最后停在哪
    plot(t(end), v(end), 'r.', 'MarkerSize', 15); 
    
    title('(a) Linear Velocity', 'FontSize', 12);
    xlabel('Time (s)', 'FontSize', 12); 
    ylabel('Velocity (m/s)', 'FontSize', 12);
    grid on; 
    
    % --- 设置坐标轴范围 ---
    xlim(x_limit); % 应用 X 轴留白
    
    % Y轴留白
    max_v = max(v);
    if max_v == 0, max_v = 1; end 
    ylim([0, max_v * 1.2]); 
    set(gca, 'FontSize', 10, 'LineWidth', 1.0);

    % ---------------------------------------------------------
    % 2. 角速度图 (Angular Velocity)
    % ---------------------------------------------------------
    figure('Name', 'Angular Velocity', 'Color', 'w', 'NumberTitle', 'off'); 
    plot(t, w, 'r-', 'LineWidth', 2.0); 
    hold on;
    plot(t(end), w(end), 'b.', 'MarkerSize', 15);
    
    title('(b) Angular Velocity', 'FontSize', 12);
    xlabel('Time (s)', 'FontSize', 12); 
    ylabel('Angular Vel (rad/s)', 'FontSize', 12);
    grid on; 
    
    % --- 设置坐标轴范围 ---
    xlim(x_limit); % 应用 X 轴留白
    
    % Y轴对称留白
    max_val_w = max(abs(w));
    if max_val_w == 0, max_val_w = 0.1; end
    ylim([-max_val_w * 1.2, max_val_w * 1.2]); 
    set(gca, 'FontSize', 10, 'LineWidth', 1.0);

    % ---------------------------------------------------------
    % 3. 航向角图 (Heading Attitude)
    % ---------------------------------------------------------
    figure('Name', 'Heading Attitude', 'Color', 'w', 'NumberTitle', 'off'); 
    plot(t, theta, 'k-', 'LineWidth', 2.0);
    hold on;
    plot(t(end), theta(end), 'r.', 'MarkerSize', 15);
    
    title('(c) Heading Attitude', 'FontSize', 12);
    xlabel('Time (s)', 'FontSize', 12); 
    ylabel('Heading (rad)', 'FontSize', 12);
    grid on; 
    
    % --- 设置坐标轴范围 ---
    xlim(x_limit); % 应用 X 轴留白
    
    % Y轴自适应留白
    min_h = min(theta); max_h = max(theta);
    range_h = max_h - min_h;
    if range_h == 0, range_h = 1; end
    ylim([min_h - range_h*0.1, max_h + range_h*0.1]); 
    set(gca, 'FontSize', 10, 'LineWidth', 1.0);
end

