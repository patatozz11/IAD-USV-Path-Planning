function [Final_Path, metrics] = DWA_Fusion_noLOS(gridMap, obs_Static, Obs_Unknown, moveobs_trajectory, Path, start, goal, Kinematic, evalParam)
    %% 1. Initialization
    global dt;     
    obstacleR = 0.5; 
    
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
    h_current_target = plot(NaN, NaN, 'r*', 'MarkerSize', 10, 'LineWidth', 1.5);
    if ~isempty(videoWriter)
        drawnow;
        writeVideo(videoWriter, getframe(videoFigure));
    end
    
    %% Data recording arrays
    % Store [time, linear velocity, angular velocity, heading] at each step.
    hist_time = [];
    hist_v = [];
    hist_w = [];
    hist_theta = [];
    
    %% Initialize path tracking index
    path_idx = 2; 
    reach_dist = 1; 
    timerStart = tic;
    success = false;
    real_navigation_time = NaN;
    path_length = NaN;
    computation_time = NaN;
    
    %% 3. Main simulation loop
    MaxSteps = 5000;
    
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
        
        % --- C. Select the next local target without LOS ---
        current_target = Path(min(path_idx, size(Path, 1)), :);
        dist_to_target = norm(x(1:2)' - current_target);
        
        if dist_to_target < reach_dist && path_idx < size(Path, 1)
            path_idx = path_idx + 1; 
            current_target = Path(path_idx, :); 
        end
        local_goal = current_target;
        set(h_current_target, 'XData', local_goal(1)+0.5, 'YData', local_goal(2)+0.5);
        
        % --- D. DWA core calculation ---
        [u, traj_candidates] = DynamicWindowApproach(x, Kinematic, local_goal, evalParam, obstacle, obstacleR);
        
        % --- E. Motion update ---
        x = f(x, u);
        traj = [traj; x(1:2)'];
        
        % Record the current state.
        hist_time = [hist_time; i * dt];     % Current time.
        hist_v = [hist_v; x(4)];             % Linear velocity v.
        hist_w = [hist_w; x(5)];             % Angular velocity w.
        hist_theta = [hist_theta; x(3)];     % Heading angle theta.
        
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
        
        % --- G. Check whether the final goal is reached ---
        dist_to_final_goal = sqrt((x(1)-goal(1))^2 + (x(2)-goal(2))^2);
        if dist_to_final_goal < 0.5
            success = true;
            real_navigation_time = i * dt; 
            % Calculate actual path length.
            % diff(traj) gives the displacement between adjacent trajectory points.
            % The Euclidean norm of each displacement gives the step length.
            % Sum all step lengths to obtain the total trajectory length.
            path_length = sum(sqrt(sum(diff(traj).^2, 2)));
            computation_time = toc(timerStart);
            
            % Plot motion curves after reaching the goal.
            PlotMotionCurves(hist_time, hist_v, hist_w, hist_theta);
            
            break;
        end
    end
    if isnan(real_navigation_time)
        real_navigation_time = numel(hist_time) * dt;
    end
    if isnan(computation_time)
        computation_time = toc(timerStart);
    end
    if isnan(path_length)
        path_length = sum(sqrt(sum(diff(traj).^2, 2)));
    end
    Final_Path = traj;
    metrics = buildFusionMetrics(traj, hist_w, gridMap, Obs_Unknown, moveobs_trajectory, ...
        success, real_navigation_time, computation_time, path_length);
    printFusionMetrics(metrics);
end



function metrics = buildFusionMetrics(traj, hist_w, gridMap, Obs_Unknown, moveobs_trajectory, success, navigation_time, computation_time, path_length)
    metrics = struct();
    metrics.success = success;
    metrics.path_length = path_length;
    metrics.navigation_time = navigation_time;
    metrics.computation_time = computation_time;

    [metrics.minimum_obstacle_distance, metrics.average_obstacle_distance, metrics.obstacle_distance_history] = ...
        trajectoryObstacleBoundaryDistance(traj, gridMap, Obs_Unknown, moveobs_trajectory);

    if numel(hist_w) < 2
        metrics.cumulative_delta_angular_velocity = NaN;
    else
        delta_w = abs(diff(hist_w));
        metrics.cumulative_delta_angular_velocity = sum(delta_w);
    end
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

%% Plotting helper function
function PlotMotionCurves(t, v, w, theta)
    % Prepare common plotting limits with 10 percent extra margin.
    max_t = max(t);
    if max_t == 0, max_t = 1; end
    x_limit = [0, max_t * 1.1]; % Add 10 percent margin to the x-axis.
    
    % ---------------------------------------------------------
    % 1. Linear velocity plot
    % ---------------------------------------------------------
    figure('Name', 'Linear Velocity', 'Color', 'w', 'NumberTitle', 'off'); 
    plot(t, v, 'b-', 'LineWidth', 2.0); 
    hold on;
    % Mark the final sample point.
    plot(t(end), v(end), 'r.', 'MarkerSize', 15); 
    
    title('(a) Linear Velocity', 'FontSize', 12);
    xlabel('Time (s)', 'FontSize', 12); 
    ylabel('Velocity (m/s)', 'FontSize', 12);
    grid on; 
    
    % --- Set axis limits ---
    xlim(x_limit); % Apply x-axis margin.
    
    % Add y-axis margin.
    max_v = max(v);
    if max_v == 0, max_v = 1; end 
    ylim([0, max_v * 1.2]); 
    set(gca, 'FontSize', 10, 'LineWidth', 1.0);

    % ---------------------------------------------------------
    % 2. Angular velocity plot
    % ---------------------------------------------------------
    figure('Name', 'Angular Velocity', 'Color', 'w', 'NumberTitle', 'off'); 
    plot(t, w, 'r-', 'LineWidth', 2.0); 
    hold on;
    plot(t(end), w(end), 'b.', 'MarkerSize', 15);
    
    title('(b) Angular Velocity', 'FontSize', 12);
    xlabel('Time (s)', 'FontSize', 12); 
    ylabel('Angular Vel (rad/s)', 'FontSize', 12);
    grid on; 
    
    % --- Set axis limits ---
    xlim(x_limit); % Apply x-axis margin.
    
    % Add symmetric y-axis margin.
    max_val_w = max(abs(w));
    if max_val_w == 0, max_val_w = 0.1; end
    ylim([-max_val_w * 1.2, max_val_w * 1.2]); 
    set(gca, 'FontSize', 10, 'LineWidth', 1.0);

    % ---------------------------------------------------------
    % 3. Heading attitude plot
    % ---------------------------------------------------------
    figure('Name', 'Heading Attitude', 'Color', 'w', 'NumberTitle', 'off'); 
    plot(t, theta, 'k-', 'LineWidth', 2.0);
    hold on;
    plot(t(end), theta(end), 'r.', 'MarkerSize', 15);
    
    title('(c) Heading Attitude', 'FontSize', 12);
    xlabel('Time (s)', 'FontSize', 12); 
    ylabel('Heading (rad)', 'FontSize', 12);
    grid on; 
    
    % --- Set axis limits ---
    xlim(x_limit); % Apply x-axis margin.
    
    % Add adaptive y-axis margin.
    min_h = min(theta); max_h = max(theta);
    range_h = max_h - min_h;
    if range_h == 0, range_h = 1; end
    ylim([min_h - range_h*0.1, max_h + range_h*0.1]); 
    set(gca, 'FontSize', 10, 'LineWidth', 1.0);
end

