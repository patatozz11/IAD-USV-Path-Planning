clc;
clear;
close all;
%% -------------------- 1) Map --------------------
gridMap = [
     1 1 1 1 1 1 1 1 0 0 1 1 0 0 0 0 0 0 1 1;
     1 1 1 1 1 1 1 0 0 0 1 0 0 0 1 0 0 0 0 1;
     1 1 1 1 1 1 0 0 0 0 0 0 0 1 1 0 0 0 0 0;
     1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0;
     1 1 1 1 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0;
     1 1 1 0 0 0 0 0 0 0 1 0 0 0 1 0 0 0 0 0;
     1 1 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 1 1;
     1 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 1 1;
     0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     1 1 0 0 0 0 0 0 0 0 1 1 0 0 1 1 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0;
     0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1;
     0 0 0 0 1 1 1 0 0 0 0 0 0 0 1 0 0 0 1 1;
     0 0 0 0 1 1 1 0 0 0 1 0 0 0 0 0 0 1 1 1;
     0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 1 1 1 1;
     1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1;
     1 1 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1;
     1 0 0 0 0 1 0 0 0 0 0 0 0 1 1 1 1 1 1 1;
     0 0 0 0 1 1 1 0 0 1 1 0 1 1 1 1 1 1 1 1
];


% Rotate the map so matrix indexing matches the displayed grid.
gridMap = rot90(gridMap, 3);            % Rotated map used by all following operations.
MaxRow = size(gridMap, 1);              % Maximum x/grid-row index after rotation.
MaxCol = size(gridMap, 2);              % Maximum y/grid-column index after rotation.
%% -------------------- 2) Plot map --------------------
mainFigure = figure('Color','w'); hold on; axis equal;
axis([1 MaxRow+1, 1 MaxCol+1]);
set(gca, ...
    'XTick', 1:1:MaxRow+1, ...
    'YTick', 1:1:MaxCol+1, ...
    'XGrid', 'on', ...
    'YGrid', 'on', ...
    'GridLineStyle', '-', ...
    'XTickLabelRotation', 0, ...
    'FontSize', 14);

% Draw static obstacles.
[obs_static_rows, obs_static_cols] = find(gridMap == 1);
for k = 1:length(obs_static_rows)
    fill([obs_static_rows(k), obs_static_rows(k)+1, obs_static_rows(k)+1, obs_static_rows(k)], ...
         [obs_static_cols(k), obs_static_cols(k), obs_static_cols(k)+1, obs_static_cols(k)+1], 'k', 'EdgeColor', 'none');
end
obs_Static = [obs_static_rows, obs_static_cols]; 
%% -------------------- 3) Pick points --------------------
goal = pickPointOnGrid('Left-click to select the goal point (Goal)', MaxRow, MaxCol);
plot(goal(1)+.5, goal(2)+.5, 'go', 'LineWidth', 2, 'MarkerSize', 8);
text(goal(1)+0.7, goal(2)+0.7, 'End', ...
    'FontSize', 12, 'FontWeight', 'bold');

start = pickPointOnGrid('Left-click to select the start point (Start)', MaxRow, MaxCol);
plot(start(1)+.5, start(2)+.5, 'b^', 'LineWidth', 2, 'MarkerSize', 8);
text(start(1)+0.9, start(2)+0.3, 'Start', ...
    'FontSize', 12, 'FontWeight', 'bold');

% simple check
if isObstacle(start, gridMap) || isObstacle(goal, gridMap)
    errordlg('Start or goal is on an obstacle. Please rerun and select free cells.');
    return;
end
%% -------------------- 4) Run A* --------------------
% diagRule:
% 0 = allow diagonal corner cutting.
% 1 = strict anti-corner-cutting rule.
% 2 = relaxed anti-corner-cutting rule.

diagRule = 1;
%[Path, distanceX, OPEN_num, ~, run_time] = Astar_Conventional8(gridMap, start, goal, diagRule);
[Path, distanceX, OPEN_num, ~, run_time] = IAstar_FiveDir_Fallback_Fast(gridMap, start, goal, diagRule);
%% -------------------- 5) Draw path --------------------
if isempty(Path)
    title(sprintf('No path | OPEN: %d | %.2f ms', OPEN_num, run_time), 'FontSize', 12);
    return;
end

plot(Path(:,1)+.5, Path(:,2)+.5, 'b:', 'LineWidth', 2);
% Optional black dashed reference path.
%plot(Path(:,1)+.5, Path(:,2)+.5, 'k--', 'LineWidth', 1.5);
fprintf('Length: %.2f | OPEN nodes: %d | Time: %.2f ms | diagRule=%d\n', ...
    distanceX, OPEN_num, run_time, diagRule);
%% -------------------- 6) Moving Obstacle Setup --------------------
% Select the start and goal of the moving obstacle.
moveObs_Start = pickPointOnGrid('Select the moving obstacle start point (Moving Obs Start)', MaxRow, MaxCol);
plot(moveObs_Start(1)+0.5, moveObs_Start(2)+0.5, 'k^', 'MarkerSize', 10, 'LineWidth', 2);
%text(moveObs_Start(1)+0.7, moveObs_Start(2)+0.7, 'moveObs Start');

moveObs_Goal = pickPointOnGrid('Select the moving obstacle goal point (Moving Obs Goal)', MaxRow, MaxCol);
plot(moveObs_Goal(1)+0.5, moveObs_Goal(2)+0.5, 'ko', 'MarkerSize', 10, 'LineWidth', 2);
%text(moveObs_Goal(1)+0.7, moveObs_Goal(2)+0.7, 'moveObs Goal');

% Plan the path of the moving obstacle using conventional A*.
[path_MoveObs, ~, ~, ~, ~] = IAstar_FiveDir_Fallback_Fast(gridMap, moveObs_Start, moveObs_Goal, diagRule);

if ~isempty(path_MoveObs)
    % Generate an interpolated trajectory for the moving obstacle.
    % v_obsmove is the moving distance per simulation step.
    % For dt = 0.1 s, v_obsmove = 0.05 corresponds to 0.5 m/s.
    v_obsmove = 0.05; 
    
    % The generated trajectory is an N-by-2 list of dense trajectory points.
    moveobs_trajectory = Line_obsMove(path_MoveObs, v_obsmove);
    
    % Draw the moving obstacle trajectory.
    plot(moveobs_trajectory(:,1)+0.5, moveobs_trajectory(:,2)+0.5, 'r-', 'LineWidth', 1);
    disp('Moving obstacle path generated.');
else
    moveobs_trajectory = [];
    disp('Moving obstacle cannot reach the target point.');
end
%% -------------------- 7) Unknown Static Obstacles Setup --------------------
% Set unknown static obstacles for local DWA avoidance.
% These obstacles are not included in GridMap and are unknown to A*.
uiwait(msgbox('Left-click to add unknown obstacles; right-click to finish.', 'Add unknown obstacles'), 2);
Obs_Unknown = []; % Unknown obstacle coordinates [x, y].
but = 1;
while but == 1
    [xval, yval, but] = ginput(1);
    if but == 1
        xval = floor(xval);
        yval = floor(yval);
        
        % Keep the selected point inside the map.
        if xval>=1 && xval<=MaxRow && yval>=1 && yval<=MaxCol
            % Draw a gray obstacle cell.
            fill([xval, xval+1, xval+1, xval], [yval, yval, yval+1, yval+1], ...
                 [0.5 0.5 0.5], 'EdgeColor', 'k'); 
            % Add it to the unknown obstacle list.
            Obs_Unknown = [Obs_Unknown; xval, yval]; 
        end
    end
end
disp(['Added ', num2str(size(Obs_Unknown, 1)), ' unknown obstacles.']);

% Create the legend only after all points and obstacles have been selected.
hLegendDynamicObs = plot(NaN, NaN, 's', ...
    'MarkerSize', 10, 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', ...
    'LineStyle', 'none');
hLegendUnknownObs = plot(NaN, NaN, 's', ...
    'MarkerSize', 10, 'MarkerFaceColor', [0.5 0.5 0.5], 'MarkerEdgeColor', 'k', ...
    'LineStyle', 'none');
hLegendLocalTarget = plot(NaN, NaN, 'r*', ...
    'MarkerSize', 10, 'LineWidth', 1.5, 'LineStyle', 'none');
hLegendMoveGoal = plot(NaN, NaN, 'ko', ...
    'MarkerSize', 10, 'LineWidth', 2, 'LineStyle', 'none');
hLegendMoveStart = plot(NaN, NaN, 'k^', ...
    'MarkerSize', 10, 'LineWidth', 2, 'LineStyle', 'none');

hLegend = legend([hLegendDynamicObs, hLegendUnknownObs, hLegendLocalTarget, ...
                  hLegendMoveGoal, hLegendMoveStart], ...
    {'Dynamic obstacle', 'Unknown static obstacle', 'Local target', ...
     'Moving-obstacle goal', 'Moving-obstacle start'}, ...
    'Location', 'northwest', 'FontSize', 13, 'Box', 'on');
set(hLegend, 'AutoUpdate', 'off');
drawnow;

% Pause here so the legend can be dragged before animation and recording.
hLegendPrompt = msgbox( ...
    'Drag the legend to the desired position, then click OK to start recording.', ...
    'Adjust legend position');
uiwait(hLegendPrompt);
figure(mainFigure);
drawnow;
%% -------------------- 8) DWA Setup --------------------
% Initialize DWA parameters.
global dt; 
dt = 0.1;
% Kinematic model: [max speed, max yaw rate, acceleration, yaw acceleration, speed resolution, yaw-rate resolution].
Kinematic=[1.0,toRadian(20.0),0.2,toRadian(50.0),0.01,toRadian(1)];  
% Evaluation parameters: [heading, distance, velocity, prediction time].
evalParam = [0.05, 0.2, 0.1, 3.0]; 

global GRIDMAP_FOR_LG;
GRIDMAP_FOR_LG = gridMap;
useLOS = true;   % false: IA* + Floyd + traditional DWA
                  % true : IA* + Floyd + LOS + traditional DWA

% Automatically record the DWA animation after all interactive selections.
recordVideo = true;
videoFile = fullfile(pwd, ...
    ['fusion_AstarDWA_recording_', datestr(now, 'yyyymmdd_HHMMSS'), '.mp4']);
videoWriter = [];
videoFigure = mainFigure;
if recordVideo
    videoWriter = VideoWriter(videoFile, 'MPEG-4');
    videoWriter.FrameRate = round(1 / dt);
    videoWriter.Quality = 95;
    open(videoWriter);
    setappdata(videoFigure, 'DWA_VideoWriter', videoWriter);
    fprintf('Video recording started: %s\n', videoFile);
end

try
    if useLOS
        clear DWA_Fusion_LOS_Traditional;
        Final_Path = DWA_Fusion_LOS(gridMap,obs_Static,Obs_Unknown,moveobs_trajectory,Path,start,goal,Kinematic,evalParam);
    else
        Final_Path = DWA_Fusion_noLOS(gridMap,obs_Static,Obs_Unknown,moveobs_trajectory,Path,start,goal,Kinematic,evalParam);
    end
catch ME
    if recordVideo
        if isgraphics(videoFigure) && isappdata(videoFigure, 'DWA_VideoWriter')
            rmappdata(videoFigure, 'DWA_VideoWriter');
        end
        try
            close(videoWriter);
        catch
        end
    end
    rethrow(ME);
end

if recordVideo
    if isgraphics(videoFigure) && isappdata(videoFigure, 'DWA_VideoWriter')
        rmappdata(videoFigure, 'DWA_VideoWriter');
    end
    close(videoWriter);
    fprintf('Video saved: %s\n', videoFile);
end
%% ==================== helper functions ====================
function p = pickPointOnGrid(promptStr, MaxRow, MaxCol)
    uiwait(msgbox(promptStr, 'Prompt'), 2);
    but = 0;
    while but ~= 1
        [xval, yval, but] = ginput(1);
    end
    xval = floor(xval);
    yval = floor(yval);

    % clamp to valid range
    xval = max(1, min(MaxRow, xval)); % x corresponds to the displayed row index.
    yval = max(1, min(MaxCol, yval)); % y corresponds to the displayed column index.
    p = [xval, yval];
end

function tf = isObstacle(p, gridMap)
    tf = (gridMap(p(1), p(2)) == 1);
end

function radian = toRadian(degree)    
    radian = degree/180*pi;  
end
  


