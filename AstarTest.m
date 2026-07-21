clear all;
clc; 

%% -------------------- 1) Map --------------------
%  U型障碍
% gridMap = [
% 0 0 0 0 0 0 0 0 0 ;
% 0 0 0 0 0 0 0 0 0 ;
% 0 0 1 0 0 0 1 0 0 ;
% 0 0 1 0 0 0 1 0 0 ;
% 0 0 1 0 0 0 1 0 0 ;
% 0 0 1 0 0 0 1 0 0 ;
% 0 0 1 1 1 1 1 0 0 ;
% 0 0 0 0 0 0 0 0 0 ; 
% 0 0 0 0 0 0 0 0 0 
% ];

%高度受限的狭窄通道


% gridMap = [
% 0 0 0 0 0 0 0 0 0;
% 0 0 0 0 0 0 0 0 0;
% 0 1 1 1 1 1 1 0 0;
% 0 1 0 0 0 0 0 0 0;
% 0 1 0 1 1 1 1 0 0;
% 0 1 0 0 0 1 1 0 0;
% 0 1 1 1 1 0 0 0 0;
% 0 0 0 0 0 0 0 0 0;
% 0 0 0 0 0 0 0 0 0
% ];




gridMap = [
        1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 1 1 0;
        1 1 1 1 1 1 1 1 0 0 0 0 0 1 0 0 0 1 1 0;
        1 1 1 1 1 1 1 0 0 0 0 0 0 1 0 0 0 0 0 0;
        1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
        1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0;
        1 1 1 1 0 1 1 0 0 0 1 1 0 0 0 0 0 0 0 0;
        1 1 1 0 0 0 0 0 0 0 1 1 0 0 0 1 1 0 0 0;
        1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
        1 0 0 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 0 0;
        0 1 1 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 1 1 0 0 0 1 0 0 0 0 0 0 1;
        0 0 0 0 0 0 0 0 0 0 0 0 1 0 1 1 0 0 1 1;
        0 1 1 0 0 0 0 0 0 0 0 0 0 0 1 1 0 1 1 1;
        0 1 1 0 0 0 1 1 1 0 0 0 0 0 0 0 1 1 1 1;
        0 0 0 0 0 0 1 1 1 0 0 1 1 0 0 0 1 1 1 1;
        0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 1 1 1 1 1;
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1;
        0 0 0 0 0 1 1 1 0 0 0 0 0 1 1 1 1 1 1 1;
        0 0 0 0 0 1 1 0 0 0 0 0 1 1 1 1 1 1 1 1;
        0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1
    ];
% gridMap = [
%     1 1 1 1 1 1 1 1 1 0 1 0 1 0 0 0 0 0 0 0;
%     1 1 1 1 1 1 1 1 0 0 0 0 1 1 0 1 1 0 0 0;
%     1 1 1 1 1 1 1 0 0 0 1 0 0 0 0 0 0 0 0 0;
%     1 1 1 1 1 1 1 0 0 0 0 0 0 1 1 0 0 0 1 0;
%     1 1 1 1 1 0 0 1 1 0 0 1 1 1 0 1 0 0 0 0;
%     1 1 1 1 0 1 1 0 0 0 1 0 0 0 0 0 0 1 0 0;
%     1 1 1 0 0 1 0 0 1 0 1 1 0 0 0 0 0 0 0 1;
%     1 1 0 0 1 0 0 0 0 1 1 0 0 0 1 0 0 1 0 0;
%     1 0 0 0 0 0 1 0 0 1 0 0 1 0 0 0 0 0 0 0;
%     0 0 0 0 0 0 0 0 1 0 0 1 0 1 0 0 0 0 0 0;
%     0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 1;
%     0 1 0 1 0 0 0 0 1 0 1 0 0 1 1 0 0 0 1 1;
%     0 0 0 0 0 1 0 0 0 0 0 1 0 1 0 0 0 1 1 1;
%     0 0 1 1 0 1 0 1 0 0 0 0 0 0 0 0 1 1 1 1;
%     0 0 1 0 0 0 0 0 1 0 0 1 0 0 1 1 1 1 1 1;
%     0 0 0 0 1 1 0 0 0 1 0 0 0 0 1 1 1 1 1 1;
%     0 0 0 0 1 0 0 0 0 0 0 1 0 1 1 1 1 1 1 1;
%     0 0 0 0 0 0 0 1 1 0 0 0 1 1 1 1 1 1 1 1;
%     0 1 0 0 0 1 0 0 0 0 0 1 1 1 1 1 1 1 1 1;
%     0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1
% ];

% gridMap = [
%      1 1 0 0 0 1 1 1 0 0 1 1 0 0 0 0 0 0 1 1;
%      1 0 0 0 0 0 0 0 0 0 1 0 0 0 1 0 0 0 0 1;
%      0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0;
%      0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 1 0 0;
%      1 1 0 0 0 1 1 0 0 0 1 0 0 0 0 0 0 0 0 0;
%      0 0 0 0 0 0 0 0 0 0 1 0 0 0 1 0 0 0 0 0;
%      0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 1 1;
%      0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 1 1;
%      0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
%      0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
%      1 1 0 0 0 0 0 0 0 0 1 1 0 0 1 1 0 0 0 1;
%      0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 1;
%      0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
%      0 0 0 0 1 1 1 0 0 0 0 0 0 0 1 0 0 0 0 0;
%      0 0 0 0 1 1 1 0 0 0 1 0 0 0 0 0 0 1 0 0;
%      0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 0 1 1 0;
%      1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0;
%      1 1 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0;
%      1 0 0 0 0 1 0 0 0 0 0 0 0 1 1 0 0 0 0 0;
%      0 0 0 0 1 1 1 0 0 1 1 0 0 0 0 0 0 0 1 1
% ];



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

k = 1;
for i = 1:MaxRow        
    for j = 1:MaxCol
        if gridMap(i,j) == 1
            fill([i,i+1,i+1,i],[j,j,j+1,j+1],'k','EdgeColor','none');
            k = k + 1;
        end
    end
end

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
%[Path, distanceX, OPEN_num, OPEN, run_time] = Astar_improved_expand(gridMap, start, goal, 1);
[Path, distanceX, OPEN_num, OPEN, run_time] = Astar_Conventional8(gridMap, start, goal, 0); %这里的0是diagRule = 0；
%[Path, distanceX, OPEN_num, OPEN, run_time] = IAstar_FiveDir_Fallback(gridMap, start, goal, 1); % %这里的1是diagRule = 1；
%[Path, distanceX, OPEN_num, OPEN, run_time] = IAstar_FiveDir_Fallback_Fast(gridMap, start, goal, 1); 

%% -------------------- 5) Draw OPEN region (gray) --------------------
% 论文出图一般建议只画 OPEN(flag==1)，更像“搜索区域”
showOpenGray = true;   % true=画所有OPEN灰色；false=不画OPEN，只画路径 
if showOpenGray && ~isempty(OPEN)
    for i = 1:size(OPEN,1)
        x = OPEN(i,2);
        y = OPEN(i,3);
        fill([x,x+1,x+1,x], [y,y,y+1,y+1], [0.4 0.4 0.4], ...
            'EdgeColor','none', 'FaceAlpha',0.25);
    end
end
%% -------------------- 6) Draw path & numbering --------------------
if isempty(Path)
    title(sprintf('No path | OPEN: %d | %.2f ms', OPEN_num, run_time), 'FontSize', 12);
    return;
end

sampleStep = 0.05;
[minObsDist, avgObsDist] = calc_path_obstacle_distance(Path, gridMap, sampleStep);

% true=画节点编号，false不画 
showIndex = false;
if showIndex
    for i = 2:size(Path,1)-1
        text(Path(i,1)+0.6, Path(i,2)+0.8, sprintf('%d', i-1), ...
            'FontSize', 14, 'Color', 'k', 'HorizontalAlignment', 'center');
    end
end

plot(Path(:,1)+.5, Path(:,2)+.5, 'b:', 'LineWidth', 2);
%plot(Path(:,1)+.5, Path(:,2)+.5, 'b.', 'MarkerSize', 14);   %和showIndex控制节点

xlabel(sprintf(['Length: %.2f | OPEN nodes: %d | Time: %.2f ms | ' ...
    'MinDist: %.3f | AvgDist: %.3f | diagRule=%d'], ...
    distanceX, OPEN_num, run_time, minObsDist, avgObsDist, diagRule), ...
    'FontSize', 11, 'FontWeight', 'bold');

fprintf('Length: %.2f | OPEN nodes: %d | Time: %.2f ms | diagRule=%d\n', ...
    distanceX, OPEN_num, run_time, diagRule);
fprintf('最小障碍物距离: %.4f grid\n', minObsDist);
fprintf('平均障碍物距离: %.4f grid\n', avgObsDist);
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


