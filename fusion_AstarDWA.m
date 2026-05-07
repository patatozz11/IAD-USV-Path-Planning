clc;
clear;
close all;
%% -------------------- 1) Map --------------------
gridMap = [
     1 1 0 0 0 1 1 1 0 0 1 1 0 0 0 0 0 0 1 1;
     1 0 0 0 0 0 0 0 0 0 1 0 0 0 1 0 0 0 0 1;
     0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0;
     0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 1 0 0;
     1 1 0 0 0 1 1 0 0 0 1 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 1 0 0 0 1 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 1 1;
     0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 1 1;
     0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     1 1 0 0 0 0 0 0 0 0 1 1 0 0 1 1 0 0 0 1;
     0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 1;
     0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 1 1 1 0 0 0 0 0 0 0 1 0 0 0 0 0;
     0 0 0 0 1 1 1 0 0 0 1 0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 0 1 1 0;
     1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0;
     1 1 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0;
     1 0 0 0 0 1 0 0 0 0 0 0 0 1 1 0 0 0 0 0;
     0 0 0 0 1 1 1 0 0 1 1 0 0 0 0 0 0 0 1 1
];

% gridMap = [
%         0 1 0 0 1 0 0 1 1 0 1 0 1 0 0 0 0 0 0 0;
%         0 1 1 0 0 1 0 0 0 0 0 0 1 1 0 1 1 0 0 0;
%         0 1 0 0 0 0 0 1 0 0 1 0 0 0 0 0 0 0 0 0;
%         0 0 0 0 0 1 1 0 0 0 0 0 0 1 1 0 0 0 1 0;
%         1 1 0 0 0 0 0 1 1 0 0 1 1 1 0 1 0 0 0 0;
%         0 1 0 0 0 1 1 0 0 0 1 0 0 0 0 0 0 1 0 0;
%         0 0 0 0 0 1 0 0 1 0 1 1 0 0 0 0 0 0 0 1;
%         0 0 1 0 0 0 0 0 0 1 1 0 0 0 1 0 0 1 0 0;
%         0 0 0 0 0 0 1 0 0 1 0 0 1 0 0 0 0 0 0 0;
%         0 0 0 0 0 0 0 0 1 0 0 1 0 1 0 0 0 0 0 0;
%         1 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0;
%         0 1 0 1 0 0 0 0 1 0 1 0 0 1 1 0 0 0 0 0;
%         0 0 0 0 0 1 0 0 0 0 0 1 0 1 0 0 0 1 0 0;
%         0 0 1 1 0 1 0 1 0 0 0 0 0 0 0 0 0 0 0 0;
%         0 0 1 0 0 0 0 0 1 0 0 1 0 0 1 0 0 0 0 0;
%         0 0 0 0 1 1 0 0 0 1 0 0 0 0 0 0 0 0 0 0;
%         0 0 0 0 1 0 0 0 0 0 0 1 0 0 1 0 0 0 0 0;
%         0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 1 0 0 0 0;
%         0 1 0 0 0 1 0 0 0 1 1 0 0 0 0 0 0 0 0 0;
%         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
%     ];

% 用 rot90(MAX0,3)来让显示和矩阵一致，这里保留
gridMap = rot90(gridMap, 3);            %旋转后的矩阵（记住，一切操作都是围绕旋转后的矩阵）
MaxRow = size(gridMap, 1);              %旋转后的矩阵Y轴能取的最大值，用来防止越界 行数，
MaxCol = size(gridMap, 2);              %旋转后的矩阵X轴能取的最大值，用来防止越界 列数，
%% -------------------- 2) Plot map --------------------
figure('Color','w'); hold on; axis equal;
axis([1 MaxRow+1, 1 MaxCol+1]);
set(gca,'XTick',1:1:MaxRow+1,'YTick',1:1:MaxCol+1,'XGrid','on','YGrid','on');
set(gca,'GridLineStyle','-');
set(gca, 'XTickLabelRotation', 0); % 强制 X 轴标签旋转角度为 0（水平）

% 绘制静态地图障碍物
[obs_static_rows, obs_static_cols] = find(gridMap == 1);
for k = 1:length(obs_static_rows)
    fill([obs_static_rows(k), obs_static_rows(k)+1, obs_static_rows(k)+1, obs_static_rows(k)], ...
         [obs_static_cols(k), obs_static_cols(k), obs_static_cols(k)+1, obs_static_cols(k)+1], 'k', 'EdgeColor', 'none');
end
obs_Static = [obs_static_rows, obs_static_cols]; 
%% -------------------- 3) Pick points --------------------
goal = pickPointOnGrid('请使用鼠标左键选择终点 (Goal)', MaxRow, MaxCol);
plot(goal(1)+.5, goal(2)+.5, 'go', 'LineWidth', 2, 'MarkerSize', 8);
text(goal(1)+0.7, goal(2)+0.7, 'End', 'FontSize', 12);

start = pickPointOnGrid('请使用鼠标左键选择起点 (Start)', MaxRow, MaxCol);
plot(start(1)+.5, start(2)+.5, 'b^', 'LineWidth', 2, 'MarkerSize', 8);
text(start(1)+0.7, start(2)+0.7, 'Start', 'FontSize', 12);

% simple check
if isObstacle(start, gridMap) || isObstacle(goal, gridMap)
    errordlg('起点或终点选在障碍物上，请重新运行并选择空白格。');
    return;
end
%% -------------------- 4) Run A* --------------------
% diagRule:
% 0 = 允许穿角
% 1 = 严格禁止穿角（相邻两格任意一个是障碍就禁止对角）
% 2 = 宽松禁止穿角（相邻两格都障碍才禁止对角）

diagRule = 1;
%[Path, distanceX, OPEN_num, ~, run_time] = Astar(gridMap, start, goal, diagRule);
[Path, distanceX, OPEN_num, ~, run_time] = Astar_improved_expand(gridMap, start, goal, diagRule);
%% -------------------- 5) Draw path --------------------
if isempty(Path)
    title(sprintf('No path | OPEN: %d | %.2f ms', OPEN_num, run_time), 'FontSize', 12);
    return;
end

plot(Path(:,1)+.5, Path(:,2)+.5, 'b:', 'LineWidth', 2);
% 改成黑色虚线 (k--)，稍微细一点，这就不会和蓝线混在一起了
%plot(Path(:,1)+.5, Path(:,2)+.5, 'k--', 'LineWidth', 1.5);
fprintf('Length: %.2f | OPEN nodes: %d | Time: %.2f ms | diagRule=%d\n', ...
    distanceX, OPEN_num, run_time, diagRule);
%% -------------------- 6) Moving Obstacle Setup --------------------
% 选择移动障碍物的起点和终点
moveObs_Start = pickPointOnGrid('请选择移动障碍物 起点 (Moving Obs Start)', MaxRow, MaxCol);
plot(moveObs_Start(1)+0.5, moveObs_Start(2)+0.5, 'k^', 'MarkerSize', 10, 'LineWidth', 2);
%text(moveObs_Start(1)+0.7, moveObs_Start(2)+0.7, 'moveObs Start');

moveObs_Goal = pickPointOnGrid('请选择移动障碍物 终点 (Moving Obs Goal)', MaxRow, MaxCol);
plot(moveObs_Goal(1)+0.5, moveObs_Goal(2)+0.5, 'ko', 'MarkerSize', 10, 'LineWidth', 2);
%text(moveObs_Goal(1)+0.7, moveObs_Goal(2)+0.7, 'moveObs Goal');

% 规划移动障碍物的路径 (复用已经写好的 Astar)
[path_MoveObs, ~, ~, ~, ~] = Astar_improved(gridMap, moveObs_Start, moveObs_Goal, diagRule);

if ~isempty(path_MoveObs)
    % 生成插值轨迹 (模拟移动)
    % v_obsmove: 每个仿真步长(dt=0.1s)移动的距离
    % 如果设为 0.05，代表速度是 0.5 m/s (假设 dt=0.1)
    v_obsmove = 0.05; 
    
    % 调用生成轨迹函数  moveobs_trajectory 是一个 Nx2 的矩阵，包含密集的轨迹点
    moveobs_trajectory = Line_obsMove(path_MoveObs, v_obsmove);
    
    % 绘制移动障碍物的轨迹 (红色细线)
    plot(moveobs_trajectory(:,1)+0.5, moveobs_trajectory(:,2)+0.5, 'r-', 'LineWidth', 1);
    disp('移动障碍物路径生成完毕。');
else
    moveobs_trajectory = [];
    disp('移动障碍物无法到达目标点！');
end
%% -------------------- 7) Unknown Static Obstacles Setup --------------------
% 设置未知静态障碍物 (模拟 DWA 局部避障能力)
% 这些障碍物不在 GridMap 中，A* 不知道它们，DWA 需要临时避开
uiwait(msgbox('请在地图上 点击左键 添加未知障碍物，点击右键 结束添加。', '添加未知障碍物'), 2);
Obs_Unknown = []; % 存储未知障碍物坐标 [x, y]
but = 1;
while but == 1
    [xval, yval, but] = ginput(1);
    if but == 1
        xval = floor(xval);
        yval = floor(yval);
        
        % 限制在地图范围内
        if xval>=1 && xval<=MaxRow && yval>=1 && yval<=MaxCol
            % 绘制灰色方块
            fill([xval, xval+1, xval+1, xval], [yval, yval, yval+1, yval+1], ...
                 [0.5 0.5 0.5], 'EdgeColor', 'k'); 
            % 加入列表
            Obs_Unknown = [Obs_Unknown; xval, yval]; 
        end
    end
end
disp(['添加了 ', num2str(size(Obs_Unknown, 1)), ' 个未知障碍物。']);
%% -------------------- 8) DWA Setup --------------------
%初始化参数
global dt; 
dt = 0.1;
% 运动学模型  [最高速度m/s, 最高旋转速度rad/s(输入是°), 加速度m/ss, 旋转加速度rad/ss, 速度分辨率m/s, 转速分辨率rad/s]  
Kinematic=[1.0,toRadian(20.0),0.2,toRadian(10.0),0.01,toRadian(1)];  
% 评价函数参数 [heading,dist,velocity,predictDT]  
evalParam = [0.05, 0.2, 0.1, 3.0]; 

global GRIDMAP_FOR_LG;
GRIDMAP_FOR_LG = gridMap;
clear LocalGoal_Strategy;   % 重置 persistent（每次仿真前建议清一下）
% 直接调用上面的函数，传入 main 里的变量
Final_Path=DWA_Fusion(gridMap,obs_Static,Obs_Unknown,moveobs_trajectory,Path,start,goal,Kinematic,evalParam);
%Final_Path=DWA_improvedFusion(gridMap,obs_Static,Obs_Unknown,moveobs_trajectory,Path,start,goal,Kinematic,evalParam);
%% ==================== helper functions ====================
function p = pickPointOnGrid(promptStr, MaxRow, MaxCol)
    uiwait(msgbox(promptStr, '提示'), 2);
    but = 0;
    while but ~= 1
        [xval, yval, but] = ginput(1);
    end
    xval = floor(xval);
    yval = floor(yval);

    % clamp to valid range
    xval = max(1, min(MaxRow, xval)); % 注意：x 对应行数所以要小等于 MaxRow
    yval = max(1, min(MaxCol, yval)); %       y 对应列数所以要小等于 MaxCol
    p = [xval, yval];
end

function tf = isObstacle(p, gridMap)
    tf = (gridMap(p(1), p(2)) == 1);
end

function radian = toRadian(degree)    
    radian = degree/180*pi;  
end
  


