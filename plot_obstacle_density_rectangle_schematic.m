clear; clc; close all;

outDir = fullfile('results', 'figures');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

mapSize = 13;
gridMap = zeros(mapSize, mapSize);

candidate = [2, 3];
goal = [11, 10];
obstacles = [3 5; 4 7; 5 9; 6 4; 6 10; 7 6; 8 8; 9 5; 10 7; 12 12; 13 8];

for i = 1:size(obstacles, 1)
    gridMap(obstacles(i,1), obstacles(i,2)) = 1;
end

xMin = min(candidate(1), goal(1));
xMax = max(candidate(1), goal(1));
yMin = min(candidate(2), goal(2));
yMax = max(candidate(2), goal(2));

nx = xMax - xMin + 1;
ny = yMax - yMin + 1;
rectRegion = gridMap(xMin:xMax, yMin:yMax);
obsCount = sum(rectRegion(:));

% --- 核心修改1：创建图窗时调整初始比例，使其更贴合网格本身的纵横比 ---
fig = figure('Color', 'w', 'Position', [100 80 820 720]);
hold on;
axis equal;

% --- 核心修改2：收紧坐标轴边界，只在网格(1~14)外围留出刚好够放文字的轻微留白 ---
% 原来是 [0, mapSize+4]，现在收紧到 [-2, mapSize+4] 和 [-1.5, mapSize+2.5] 之间平衡文字
xlim([-1.5, mapSize+4]);
ylim([-1.2, mapSize+2.5]);
axis off; 

% 1. 手动绘制底层网格
for x = 1:mapSize
    for y = 1:mapSize
        rectangle('Position', [x, y, 1, 1], 'FaceColor', 'w', ...
                  'EdgeColor', [0.72 0.72 0.72]);
    end
end

% 2. 填充黄色统计区域背景
fill([xMin xMax+1 xMax+1 xMin], [yMin yMin yMax+1 yMax+1], ...
    [1.0 0.92 0.72], 'FaceAlpha', 0.40, 'EdgeColor', 'none');

% 3. 绘制黑色障碍物
for i = 1:size(obstacles, 1)
    x = obstacles(i, 1);
    y = obstacles(i, 2);
    rectangle('Position', [x, y, 1, 1], 'FaceColor', 'k', 'EdgeColor', 'k');
end

% 4. 统计区域红色虚线外框
rectangle('Position', [xMin, yMin, nx, ny], ...
    'EdgeColor', [0.85 0.05 0.05], 'LineStyle', '--', 'LineWidth', 2.8);


% ----------------- 文本与标记位置优化 ----------------- %

% [起点]
plot(candidate(1)+0.5, candidate(2)+0.5, 'o', ...
    'MarkerEdgeColor', [0 0.15 0.95], 'MarkerFaceColor', [0 0.35 1], ...
    'MarkerSize', 14, 'LineWidth', 2.0);
text(candidate(1)+0.1, candidate(2)+0.5, '$n(x_n,y_n)$', ...
    'Color', [0 0.15 0.95], 'FontName', 'Times New Roman', ...
    'FontSize', 22, 'FontWeight', 'bold', 'Interpreter', 'latex', ...
    'HorizontalAlignment', 'right');

% [终点]
plot(goal(1)+0.5, goal(2)+0.5, 'p', ...
    'MarkerEdgeColor', [0 0.45 0], 'MarkerFaceColor', [0.1 0.85 0.1], ...
    'MarkerSize', 18, 'LineWidth', 2.0);
text(goal(1)+1.0, goal(2)+0.5, '$goal(x_{end},y_{end})$', ...
    'Color', [0 0.45 0], 'FontName', 'Times New Roman', ...
    'FontSize', 22, 'FontWeight', 'bold', 'Interpreter', 'latex', ...
    'HorizontalAlignment', 'left');

% [顶部注释] - 紧贴虚线框上方
text(xMin, yMax+1.5, 'Rectangular region', ...
    'Color', [0.75 0 0], 'FontName', 'Times New Roman', ...
    'FontSize', 20, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'left');

text(xMax+1, yMax+1.5, '$N_x=10$', ...
    'Color', [0.65 0 0], 'FontName', 'Times New Roman', ...
    'FontSize', 20, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'right', 'Interpreter', 'latex');

% [右侧注释]
centerY = (yMin + yMax + 1) / 2;
text(xMax+1.3, centerY, '$N_y=8$', ...
    'Color', [0.65 0 0], 'FontName', 'Times New Roman', ...
    'FontSize', 20, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'left', 'Interpreter', 'latex');

% [计算障碍物白叉]
countedObstacles = obstacles( ...
    obstacles(:,1) >= xMin & obstacles(:,1) <= xMax & ...
    obstacles(:,2) >= yMin & obstacles(:,2) <= yMax, :);
for i = 1:size(countedObstacles, 1)
    plot(countedObstacles(i,1)+0.5, countedObstacles(i,2)+0.5, 'x', ...
        'Color', [0.95 0.95 0.95], 'LineWidth', 2.0, 'MarkerSize', 10);
end

% [底部公式注释]
centerX = (xMin + xMax + 1) / 2;
text(centerX, yMin-0.8, sprintf('$N_{obs}(n)=%d$', obsCount), ...
    'Color', [0.05 0.05 0.05], 'FontName', 'Times New Roman', ...
    'FontSize', 20, 'FontWeight', 'bold', 'Interpreter', 'latex', ...
    'BackgroundColor', 'w', 'Margin', 2, ...
    'HorizontalAlignment', 'center');


% --- 核心修改3：强制让当前坐标轴占满整个 Figure，并消除图窗固有边缘留白 ---
ax = gca;
ax.Position = [0 0 1 1]; % 占满[左 下 宽 高]

% 导出图像
pngPath = fullfile(outDir, 'obstacle_density_rectangle_schematic_final.png');
epsPath = fullfile(outDir, 'obstacle_density_rectangle_schematic_final.eps');

% --- 核心修改4：使用 exportgraphics 导出，它会智能裁剪掉周围所有没有内容的空白 ---
exportgraphics(fig, pngPath, 'Resolution', 300, 'BackgroundColor', 'none');
exportgraphics(fig, epsPath, 'ContentType', 'vector', 'BackgroundColor', 'none');

fprintf('Saved figures with automatic tight cropping.\n');