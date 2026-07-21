function [Final_Path, metrics] = DWA_Fusion_LOS(gridMap, obs_Static, Obs_Unknown, moveobs_trajectory, Path, start, goal, Kinematic, evalParam, d_safe, losConfig)
    if nargin < 10 || isempty(d_safe)
        d_safe = 0.2;
    end
    if nargin < 11 || isempty(losConfig)
        losConfig = default_los_config();
    else
        losConfig = normalize_los_config(losConfig);
    end

    %% 1. Initialization
    global dt;     
    obstacleR = 0.5; % Obstacle inflation radius.
    
    % Initial state x = [x, y, yaw, v, w].
    if size(Path,1) > 1
        init_theta = atan2(Path(2,2)-Path(1,2), Path(2,1)-Path(1,1));
    else
        init_theta = atan2(goal(2)-start(2), goal(1)-start(1));
    end
    x = [start(1), start(2), init_theta, 0, 0]';
    
    traj = x(1:2)'; 
    obsMove_num = size(moveobs_trajectory, 1);
    
    %% 2. Plot preparation
    videoFigure = gcf;
    videoWriter = [];
    if isappdata(videoFigure, 'DWA_VideoWriter')
        videoWriter = getappdata(videoFigure, 'DWA_VideoWriter');
    end
    h_robot = plot(x(1)+0.5, x(2)+0.5, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 6);
    h_arrow = quiver(x(1)+0.5, x(2)+0.5, 0.5*cos(x(3)), 0.5*sin(x(3)), 'b', 'LineWidth', 2);
    h_dyn_fill = fill([0 0 0 0], [0 0 0 0], 'y', 'EdgeColor', 'k');
    %h_trail = plot(x(1)+0.5, x(2)+0.5, 'b.', 'MarkerSize', 2);
    % Use a red solid line for the actual trajectory.
    h_trail = plot(x(1)+0.5, x(2)+0.5, 'r-', 'LineWidth', 1);
    
    h_green_lines = [];
    % Handle for the current local target marker.
    h_current_target = plot(NaN, NaN, 'r*', 'MarkerSize', 10, 'LineWidth', 1.5);
    if ~isempty(videoWriter)
        drawnow;
        writeVideo(videoWriter, getframe(videoFigure));
    end
    
    %% Data recording
    hist_time = [];
    hist_v = [];
    hist_w = [];
    hist_theta = [];
    local_goal_history = [];
    LocalGoal_Strategy([], [], [], [], [], losConfig, true);
    
    timerStart = tic;
    
    %% 3. Main simulation loop
    MaxSteps = 5000;
    success = false;
    real_navigation_time = NaN;
    path_length = NaN;
    computation_time = NaN;
    
    for i = 1:MaxSteps
        % --- A. Get the current dynamic obstacle position ---
        curr_dyn_obs = [];
        if obsMove_num > 0
            idx = min(i, obsMove_num);
            curr_dyn_obs = moveobs_trajectory(idx, :);
            cx = curr_dyn_obs(1); cy = curr_dyn_obs(2);
            set(h_dyn_fill, 'XData', [cx, cx+1, cx+1, cx], 'YData', [cy, cy, cy+1, cy+1]);
        end
        
        % --- B. Merge static, unknown, and dynamic obstacles ---
        obstacle = [obs_Static; Obs_Unknown; curr_dyn_obs];
        
        % --- C. Select the local target using LOS ---

        % Build a local obstacle map for LOS safety checking.
        losGridMap = build_los_grid_map(gridMap, obstacle);
        local_goal = LocalGoal_Strategy(x, Path, goal, losGridMap, d_safe, losConfig, false);
        local_goal_history = [local_goal_history; local_goal];
        set(h_current_target, 'XData', local_goal(1)+0.5, 'YData', local_goal(2)+0.5);
        
        % --- D. DWA core calculation ---
        [u, traj_candidates] = DynamicWindowApproach(x, Kinematic, local_goal, evalParam, obstacle, obstacleR);
        
        % --- E. Motion update ---
        x = f(x, u);
        traj = [traj; x(1:2)'];
        
        % Record the current state.
        hist_time = [hist_time; i * dt];
        hist_v = [hist_v; x(4)];
        hist_w = [hist_w; x(5)];
        hist_theta = [hist_theta; x(3)];
        
        % --- F. Plot update ---
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
        
        if isempty(videoWriter)
            drawnow limitrate;
        else
            drawnow;
            writeVideo(videoWriter, getframe(videoFigure));
        end
        
        % --- G. Check whether the goal is reached ---
        dist_to_goal = sqrt((x(1)-goal(1))^2 + (x(2)-goal(2))^2);
        if dist_to_goal < 0.5
            success = true;
            real_navigation_time = i * dt;
            % Calculate actual path length.
            % diff(traj) gives the displacement between adjacent trajectory points.
            % The Euclidean norm of each displacement gives the step length.
            % Sum all step lengths to obtain the total trajectory length.
            path_length = sum(sqrt(sum(diff(traj).^2, 2)));
            computation_time = toc(timerStart);
            % Unified metrics are printed after the simulation loop.
            
            
            PlotMotionCurves(hist_time, hist_v, hist_w, hist_theta);
            break;
        end
    end
    if isnan(computation_time)
        computation_time = toc(timerStart);
    end
    if isnan(path_length)
        path_length = sum(sqrt(sum(diff(traj).^2, 2)));
    end
    Final_Path = traj;
    metrics = buildFusionMetrics(traj, hist_time, hist_w, hist_theta, local_goal_history, ...
        gridMap, Obs_Unknown, moveobs_trajectory, goal, success, ...
        real_navigation_time, computation_time, path_length);
    printFusionMetrics(metrics);
end



function lg = LocalGoal_Strategy(x, path, final_goal, gridMap, d_safe, losConfig, resetFlag)
    persistent k;

    if nargin >= 7 && resetFlag
        k = 1;
        lg = [];
        return;
    end
    
    % --- 1. Initialization guard ---
    % Reset k when it is uninitialized or out of range.
    if isempty(k) || k > size(path,1)
        k = 1; 
    end
    
    pos = x(1:2)'; 
    n = size(path,1);
    
    % --- 2. Edge-case handling ---
    if n == 0, lg = final_goal; return; end
    
    % Lock onto the final goal when the USV is already close to it.
    if norm(pos - final_goal) < 0.5
        lg = final_goal;
        return;
    end
    
    % --- 3. Keep k within path bounds ---
    k = min(k, n);
    
    % If the last path point is reached, keep returning the final point.
    if k == n
        lg = path(n, :);
        return;
    end
    
    % --- 4. Farthest visible safe local target selection ---
    reach_dist = losConfig.reach_dist;
    sight_dist = losConfig.sight_dist;
    max_skip = losConfig.max_skip;

    current_target = path(k, :);
    dist_to_current = norm(pos - current_target);

    % If the current target has been reached, advance at least one point.
    if dist_to_current < reach_dist
        k = min(k + 1, n);
    end

    best_k = k;
    last_idx = min(n, k + max_skip);

    % Search from far to near, and select the farthest visible safe point.
    for j = last_idx:-1:(k+1)
        candidate = path(j, :);
        if norm(pos - candidate) > sight_dist
            continue;
        end

        if isSegmentSafeWithClearance(pos, candidate, gridMap, d_safe)
            best_k = j;
            break;
        end
    end

    k = best_k;

    % --- 5. Output ---
    % Guard against out-of-range indexing after switching.
    k = min(k, n);
    lg = path(k, :);
end

function losConfig = default_los_config()
    losConfig.reach_dist = 1.0;
    losConfig.sight_dist = 4.0;
    losConfig.max_skip = 5;
end

function losConfig = normalize_los_config(losConfig)
    defaultConfig = default_los_config();
    if ~isfield(losConfig, 'reach_dist') || isempty(losConfig.reach_dist)
        losConfig.reach_dist = defaultConfig.reach_dist;
    end
    if ~isfield(losConfig, 'sight_dist') || isempty(losConfig.sight_dist)
        losConfig.sight_dist = defaultConfig.sight_dist;
    end
    if ~isfield(losConfig, 'max_skip') || isempty(losConfig.max_skip)
        losConfig.max_skip = defaultConfig.max_skip;
    end
end
%% LOS line segment safety check with grid-map clearance
function safe = isSegmentSafeWithClearance(p1, p2, gridMap, d_safe)
    safe = true;
    gridMap = logical(gridMap);
    [R, C] = size(gridMap);

    obstacleCells = obstacle_cell_list(gridMap);
    v = p2 - p1;
    segLen = norm(v);
    if segLen < 1e-9
        safe = is_point_free(p1, gridMap);
        return;
    end

    sampleStep = 0.1;
    sampleCount = max(2, ceil(segLen / sampleStep) + 1);
    for idx = 1:sampleCount
        t = (idx - 1) / (sampleCount - 1);
        p = p1 + t * v;

        ix = floor(p(1));
        iy = floor(p(2));
        if ix < 1 || ix > R || iy < 1 || iy > C || gridMap(ix, iy)
            safe = false;
            return;
        end

        if point_to_obstacle_boundary_distance(p, obstacleCells) < d_safe
            safe = false;
            return;
        end
    end
end

function ok = is_point_free(p, gridMap)
    [R, C] = size(gridMap);
    ix = floor(p(1));
    iy = floor(p(2));
    ok = ix >= 1 && ix <= R && iy >= 1 && iy <= C && ~gridMap(ix, iy);
end

function obstacleCells = obstacle_cell_list(gridMap)
    [obsX, obsY] = find(gridMap);
    obstacleCells = [obsX, obsY];
end

function minDist = point_to_obstacle_boundary_distance(p, obstacleCells)
    if isempty(obstacleCells)
        minDist = inf;
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
    minDist = min(sqrt(dx.^2 + dy.^2));
end

function losGridMap = build_los_grid_map(gridMap, obstacles)
    losGridMap = logical(gridMap);
    [R, C] = size(losGridMap);
    if isempty(obstacles)
        return;
    end

    for ii = 1:size(obstacles, 1)
        x = floor(obstacles(ii, 1));
        y = floor(obstacles(ii, 2));
        if x >= 1 && x <= R && y >= 1 && y <= C
            losGridMap(x, y) = true;
        end
    end
end


function metrics = buildFusionMetrics(traj, hist_time, hist_w, hist_theta, local_goal_history, gridMap, Obs_Unknown, moveobs_trajectory, goal, success, navigation_time, computation_time, path_length)
    metrics = struct();
    metrics.path_length = path_length;
    metrics.navigation_time = navigation_time;
    metrics.computation_time = computation_time;
    metrics.success = success;
    if isempty(traj)
        metrics.final_goal_error = NaN;
    else
        metrics.final_goal_error = norm(traj(end,:) - goal);
    end

    [metrics.minimum_obstacle_distance, metrics.average_obstacle_distance, metrics.obstacle_distance_history] = ...
        trajectoryObstacleBoundaryDistance(traj, gridMap, Obs_Unknown, moveobs_trajectory);

    if numel(hist_w) < 2
        metrics.max_delta_angular_velocity = NaN;
        metrics.cumulative_delta_angular_velocity = NaN;
        metrics.mean_delta_angular_velocity = NaN;
        metrics.RMS_delta_angular_velocity = NaN;
    else
        delta_w = abs(diff(hist_w));
        metrics.max_delta_angular_velocity = max(delta_w);
        metrics.cumulative_delta_angular_velocity = sum(delta_w);
        metrics.mean_delta_angular_velocity = mean(delta_w);
        metrics.RMS_delta_angular_velocity = sqrt(mean(delta_w.^2));
    end

    if numel(hist_theta) < 2
        metrics.max_heading_change = NaN;
        metrics.cumulative_heading_change = NaN;
    else
        delta_theta = abs(atan2(sin(diff(hist_theta)), cos(diff(hist_theta))));
        metrics.max_heading_change = max(delta_theta);
        metrics.cumulative_heading_change = sum(delta_theta);
    end

    if size(local_goal_history, 1) < 2
        metrics.target_switches = 0;
    else
        metrics.target_switches = sum(any(diff(local_goal_history, 1, 1) ~= 0, 2));
    end
    metrics.time_history = hist_time;
    metrics.angular_velocity_history = hist_w;
    metrics.heading_history = hist_theta;
    metrics.local_goal_history = local_goal_history;
end

function printFusionMetrics(metrics)
    fprintf('\n========== Main Evaluation Metrics ==========\n');
    fprintf('Success: %d\n', double(metrics.success));
    fprintf('Actual path length: %.3f m\n', metrics.path_length);
    fprintf('Actual navigation time: %.3f s\n', metrics.navigation_time);
    fprintf('Minimum obstacle distance: %.3f grid\n', metrics.minimum_obstacle_distance);
    fprintf('Average obstacle distance: %.3f grid\n', metrics.average_obstacle_distance);
    fprintf('Cumulative angular velocity change: %.5f rad/s\n', metrics.cumulative_delta_angular_velocity);
end

function [minDist, avgDist, distHistory] = trajectoryObstacleBoundaryDistance(traj, gridMap, Obs_Unknown, moveobs_trajectory)
    if isempty(traj)
        minDist = NaN;
        avgDist = NaN;
        distHistory = [];
        return;
    end

    gridMapForDistance = buildGridMapForDistance(gridMap, Obs_Unknown);
    sampleStep = 0.05;
    [minDist, avgDist, distHistory] = calc_path_obstacle_distance( ...
        traj(:,1:2), gridMapForDistance, sampleStep);
end

function gridMapForDistance = buildGridMapForDistance(gridMap, Obs_Unknown)
    gridMapForDistance = gridMap;
    if isempty(Obs_Unknown)
        return;
    end

    for ii = 1:size(Obs_Unknown, 1)
        r = round(Obs_Unknown(ii, 1));
        c = round(Obs_Unknown(ii, 2));
        if r >= 1 && r <= size(gridMapForDistance, 1) && ...
                c >= 1 && c <= size(gridMapForDistance, 2)
            gridMapForDistance(r, c) = 1;
        end
    end
end

%% Motion curve plotting
function PlotMotionCurves(t, v, w, theta)
    % Prepare common plotting limits with 10 percent extra margin.
    max_t = max(t);
    if max_t == 0, max_t = 1; end
    x_limit = [0, max_t * 1.1]; 
    
    % --- Linear velocity plot ---
    figure('Name', 'Linear Velocity', 'Color', 'w', 'NumberTitle', 'off'); 
    plot(t, v, 'b-', 'LineWidth', 2.0); hold on;
    plot(t(end), v(end), 'r.', 'MarkerSize', 15); 
    title('(a) Linear Velocity', 'FontSize', 12);
    xlabel('Time (s)', 'FontSize', 12); ylabel('Velocity (m/s)', 'FontSize', 12);
    grid on; xlim(x_limit);
    max_v = max(v); if max_v==0, max_v=1; end; ylim([0, max_v*1.2]);
    set(gca, 'FontSize', 10, 'LineWidth', 1.0);

    % --- Angular velocity plot ---
    figure('Name', 'Angular Velocity', 'Color', 'w', 'NumberTitle', 'off'); 
    plot(t, w, 'r-', 'LineWidth', 2.0); hold on;
    plot(t(end), w(end), 'b.', 'MarkerSize', 15);
    title('(b) Angular Velocity', 'FontSize', 12);
    xlabel('Time (s)', 'FontSize', 12); ylabel('Angular Vel (rad/s)', 'FontSize', 12);
    grid on; xlim(x_limit);
    max_val_w = max(abs(w)); if max_val_w==0, max_val_w=0.1; end
    ylim([-max_val_w*1.2, max_val_w*1.2]);
    set(gca, 'FontSize', 10, 'LineWidth', 1.0);

    % --- Heading attitude plot ---
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
