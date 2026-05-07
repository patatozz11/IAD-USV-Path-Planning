%% ==================== Floyd优化路径 ====================
% 路径优化（去共线 + 直连删点）
function new_path = Floyd_Smooth_safer(path, map)
    new_path = remove_colinear_points(path);

    i = 1;
    while i < size(new_path,1) - 1
        best = i + 1;  % 至少保留相邻点
        for j = i + 2 : size(new_path,1)
            if is_line_passable(new_path(i,:), new_path(j,:), map)
                best = j; % 能直连就尽量拉远
            end
        end

        if best > i + 1
            new_path(i+1:best-1,:) = []; % 删掉中间点
        end
        i = i + 1;
    end
end

% --- Step1: 去掉共线点（向量化更短） ---
function out = remove_colinear_points(p)
    n = size(p,1);
    if n < 3, out = p; return; end

    d = diff(p,1,1); % (n-1)x2
    turn = d(1:end-1,1).*d(2:end,2) - d(1:end-1,2).*d(2:end,1); % 2D叉积
    keep = [true; turn ~= 0; true];
    out = p(keep,:);
end

% --- Step2: 判断两点连线是否穿障碍（更短更快：只走过线段覆盖的格子） ---
function ok = is_line_passable(a, b, map)
    [R,C] = size(map);

    % 以“格子中心”连线（与你原来的 +0.5 一致）
    x1 = a(1) + 0.5;  y1 = a(2) + 0.5;
    x2 = b(1) + 0.5;  y2 = b(2) + 0.5;

    ix = floor(x1);  iy = floor(y1);
    ex = floor(x2);  ey = floor(y2);

    % 越界/起终点落障碍：直接不可达
    if ix<1||ix>R||iy<1||iy>C||ex<1||ex>R||ey<1||ey>C
        ok = false; return;
    end
    if map(ix,iy) || map(ex,ey)
        ok = false; return;
    end

    dx = x2 - x1;  dy = y2 - y1;
    sx = sign(dx); sy = sign(dy);

    if sx == 0
        tMaxX = inf; tDeltaX = inf;
    else
        nextVX = ix + (sx > 0);              % 下一条竖边 x = ix+1 或 x = ix
        tMaxX  = (nextVX - x1) / dx;
        tDeltaX = 1 / abs(dx);
    end

    if sy == 0
        tMaxY = inf; tDeltaY = inf;
    else
        nextHY = iy + (sy > 0);              % 下一条横边 y = iy+1 或 y = iy
        tMaxY  = (nextHY - y1) / dy;
        tDeltaY = 1 / abs(dy);
    end

    % DDA 网格遍历（supercover：角点同时跨越时，额外检查相邻格，保守不“擦边”）
    while ix ~= ex || iy ~= ey
        if tMaxX < tMaxY
            ix = ix + sx;
            if ix<1||ix>R||map(ix,iy), ok = false; return; end
            tMaxX = tMaxX + tDeltaX;

        elseif tMaxY < tMaxX
            iy = iy + sy;
            if iy<1||iy>C||map(ix,iy), ok = false; return; end
            tMaxY = tMaxY + tDeltaY;

        else
            nx = ix + sx; ny = iy + sy;

            % supercover：穿过格点(角点)时，三个格都要安全（等价于你原来“碰到边/角也不行”的判定）
            if nx<1||nx>R||ny<1||ny>C || map(nx,iy) || map(ix,ny) || map(nx,ny)
                ok = false; return;
            end

            ix = nx; iy = ny;
            tMaxX = tMaxX + tDeltaX;
            tMaxY = tMaxY + tDeltaY;
        end
    end

    ok = true;
end