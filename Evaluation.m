function [evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam)
%  Vr=[ 0.1s后最低速度 0.1s后最高速度 0.1s后最低角速度 0.1s后最高角速度 ]
% model= [最高速度[m/s], 最高旋转速度[rad/s], 加速度[m/ss], 旋转加速度[rad/ss], 速度分辨率[m/s], 转速分辨率[rad/s]]
evalDB=[];
trajDB=[];
for vt=Vr(1):model(5):Vr(2) % 最低速度：速度分辨率：最高速度
    for ot=Vr(3):model(6):Vr(4) % 最低角速度：转速分辨率：最高角速度
        % 每组[vt ot]在3秒内的运动轨迹，取31个点 traj=3s内的所有状态轨迹，xt=3s后的状态轨迹 % evalParam(4) = predictDT = 3       
        [xt,traj]=GenerateTrajectory(x,vt,ot,evalParam(4));   % [xt,traj] = [预测的当前状态，所有状态数组  ]
        % 各评价函数的计算
        heading=CalcHeadingEval(xt,goal); % 计算3s后的偏角与目标点的夹角
        dist=CalcDistEval(xt,ob,R);       % 计算3s后的位置与障碍物最小距离（有最大值限制）
        vel=abs(vt); % abs去绝对值           
        stopDist=CalcBreakingDist(vel,model); % 制动距离的计算 (安全检查)

        if dist>stopDist % 
            % 只有安全的轨迹才会被记录
            evalDB=[evalDB;[vt ot heading dist vel]]; % 预测3s后得到的[速度 转速 角度分数 距离分数 速度分数]
            trajDB=[trajDB;traj]; % trajDB = [3s内的所有状态轨迹 31个点 1-5行31列]
        end
    end
end
end

function stopDist=CalcBreakingDist(vel,model)  
    % 根据运动学模型计算制动距离  
    global dt;  
    stopDist=0;  
    while vel>0  
        stopDist=stopDist+vel*dt; 
        vel=vel-model(3)*dt; % model(3)为加速度
    end  
end

function dist=CalcDistEval(x,ob,R)  
    % 障碍物距离评价函数：越远越好
    dist=100;  
    for io=1:length(ob(:,1))  
        disttmp=norm(ob(io,:)-x(1:2)')-R;
        if dist>disttmp% 找到离最近障碍物的距离  
            dist=disttmp;  
        end  
    end  
    % 限制最大值，防止远处的障碍物权重过大
    if dist>=R  
        dist=R;  
    end  
end

function heading=CalcHeadingEval(x,goal)  
    % Heading评价函数：车头朝向目标越近越好
    theta=toDegree(x(3));                                % 当前朝向  
    goalTheta=toDegree(atan2(goal(2)-x(2),goal(1)-x(1)));% 目标点方位  
      
    if goalTheta>theta  
        targetTheta=goalTheta-theta; 
    else  
        targetTheta=theta-goalTheta;
    end  
    heading=180-targetTheta; % 误差越小，分数越高
end

function degree = toDegree(radian)    
    degree = radian/pi*180; 
end