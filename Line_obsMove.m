% === 移动障碍物轨迹生成函数  ===
function trajectory = Line_obsMove(path, v_obsMove)
    % path: A* 生成的路径点 [x, y] (Nx2)
    % v_obsMove: 每个插值点的间距 (模拟速度)
    
    if isempty(path)
        trajectory = [];
        return;
    end
    
    trajectory = path(1,:); % 初始化轨迹，放入起点
    
    % 遍历 A* 路径的每一段线段
    for i = 1:size(path,1)-1
        p1 = path(i,:);
        p2 = path(i+1,:);
        
        % 1. 计算当前线段长度 (欧氏距离)
        dist = sqrt(sum((p2 - p1).^2));
        
        % 2. 计算需要插入多少个点 (向下取整)
        num_steps = floor(dist / v_obsMove);
        
        if num_steps > 0
            % 3. 计算单位方向向量并乘以步长
            delta = (p2 - p1) / dist * v_obsMove;
            
            % 4. 生成插值点
            for k = 1:num_steps
                new_pt = p1 + delta * k;
                trajectory = [trajectory; new_pt]; %#ok<AGROW>
            end
        end
        
        % 5. 强制加入本段终点
        % 判断逻辑：如果当前轨迹最后一个点距离 p2 大于一个微小量(1e-4)，才加 p2
        % 这样既处理了除不尽的情况(补齐终点)，又防止了整除的情况(重复添加)
        
        last_pt = trajectory(end, :);
        dist_remain = sqrt(sum((p2 - last_pt).^2)); % 计算剩余距离
        
        if dist_remain > 1e-4 
            trajectory = [trajectory; p2]; 
        end
    end
end