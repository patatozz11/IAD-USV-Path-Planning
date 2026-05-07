close all;  
clear all;  

x=[0 0 0 0 0]';% 初期状态[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]  
goal=[10,10];% 目标点位置 [x(m),y(m)]  
 
obstacle=[2 2;4 5; 5 9;8 6;7 9];%设置五个障碍物
obstacleR=0.5;% 冲突判定用的障碍物排斥半径  
global dt; dt=0.1;% 时间[s]  
T = 1000;

% 运动学模型  
% [最高速度m/s, 最高旋转速度rad/s, 加速度m/ss, 旋转加速度rad/ss, 速度分辨率m/s, 转速分辨率rad/s]  
Kinematic=[1.0,toRadian(20.0),0.2,toRadian(10.0),0.01,toRadian(1)];  
  
% 评价函数参数 [heading,dist,velocity,predictDT]  
evalParam=[0.05,0.2,0.1,3.0];  
area=[-1 11 -1 11];% 模拟区域范围 [xmin xmax ymin ymax]  
  
% 模拟实验的结果  
result.x=[];  
tic;  

% Main loop  
figure('Color','w'); % 创建一个白色背景的窗口
for i=1:T
    % === 1. DWA核心：计算最佳控制量 u 和预测轨迹 traj ===
    [u,traj]=DynamicWindowApproach(x,Kinematic,goal,evalParam,obstacle,obstacleR);  
    
    % === 2. 运动更新：执行控制量，更新机器人状态 ===
    x=f(x,u);
      
    % 记录历史状态用于画图
    result.x=[result.x; x'];  
      
    % === 3. 判断是否到达目的地 ===
    if norm(x(1:2)-goal')<0.5  
        disp('Arrive Goal!!');break;  
    end  
    
    % === 4. 实时动画显示 (只看不存) ===  
    hold off;  
    ArrowLength=0.5;    
    % 画机器人当前位置和朝向
    quiver(x(1),x(2),ArrowLength*cos(x(3)),ArrowLength*sin(x(3)),'ok');hold on;  
    % 画已经走过的路径
    plot(result.x(:,1),result.x(:,2),'-b');hold on;  
    % 画目标点
    plot(goal(1),goal(2),'or');hold on;  
    % 画障碍物
    r = obstacleR;
    for id=1:length(obstacle(:,1))         
        rectangle('Position',[obstacle(id,1)-r/2,obstacle(id,2)-r/2,r,r],'Linewidth',2,'LineStyle','-','EdgeColor','y');
    end
    
    % 画 DWA 预测出来的最佳绿线 (可视化预测结果)
    if ~isempty(traj)  
        for it=1:length(traj(:,1))/5  
            ind=1+(it-1)*5;  
            plot(traj(ind,:),traj(ind+1,:),'-g');hold on;  
        end  
    end  
    
    axis(area);  
    grid on;  
    xlabel('x / m')
    ylabel('y / m')
    title('动态窗口法 DWA 仿真')
    drawnow;  % 强制刷新屏幕，产生动画效果
end 
toc  

   
%% === 子函数定义区 ===
function [u,trajDB]=DynamicWindowApproach(x,model,goal,evalParam,ob,R)  
    % Dynamic Window [vmin,vmax,wmin,wmax]  
    Vr=CalcDynamicWindow(x,model);  
      
    % 评价函数的计算  
    [evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam);  
      
    if isempty(evalDB)  
        disp('no path to goal!!');  
        u=[0;0];return;  
    end  
      
    % 各评价函数正则化  
    evalDB=NormalizeEval(evalDB);  
      
    % 最终评价函数的计算 (加权求和)
    feval=[];  
    for id=1:length(evalDB(:,1))  
        feval=[feval;evalParam(1:3)*evalDB(id,3:5)'];  
    end  
    evalDB=[evalDB feval];  
      
    [~,ind]=max(feval);% 选出分数最高的那个
    u=evalDB(ind,1:2)';% 返回最佳速度 [v, w]
end
  
function [evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam)  
    % 评价函数：遍历所有可能的速度，推演轨迹并打分
    evalDB=[];  
    trajDB=[];  
    for vt=Vr(1):model(5):Vr(2)  
        for ot=Vr(3):model(6):Vr(4)  
            % 轨迹推测; 得到 xt: 向前运动后的预测位姿; traj: 预测轨迹  
            [xt,traj]=GenerateTrajectory(x,vt,ot,evalParam(4)); 
            
            % 各评价函数的计算  
            heading=CalcHeadingEval(xt,goal);  
            dist=CalcDistEval(xt,ob,R);  
            vel=abs(vt);  
            
            % 制动距离的计算 (安全检查)
            stopDist=CalcBreakingDist(vel,model);  
            if dist>stopDist 
                % 只有安全的轨迹才会被记录
                evalDB=[evalDB;[vt ot heading dist vel]];  
                trajDB=[trajDB;traj];  
            end  
        end  
    end  
end
  
function EvalDB=NormalizeEval(EvalDB)  
    % 评价函数正则化 (归一化到 0~1 之间)
    if sum(EvalDB(:,3))~=0  
        EvalDB(:,3)=EvalDB(:,3)/sum(EvalDB(:,3));  
    end  
    if sum(EvalDB(:,4))~=0  
        EvalDB(:,4)=EvalDB(:,4)/sum(EvalDB(:,4));  
    end  
    if sum(EvalDB(:,5))~=0  
        EvalDB(:,5)=EvalDB(:,5)/sum(EvalDB(:,5));  
    end  
end
  
function [x,traj]=GenerateTrajectory(x,vt,ot,evaldt)  
    % 轨迹生成函数：根据当前速度，推演未来一段时间的位置 
    global dt;  
    time=0;  
    u=[vt;ot];
    traj=x;
    while time<=evaldt  
        time=time+dt;
        x=f(x,u);
        traj=[traj x];  
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
    if dist>=2*R  
        dist=2*R;  
    end  
end

function heading=CalcHeadingEval(x,goal)  
    % Heading评价函数：车头朝向目标越近越好
    theta=toDegree(x(3));% 机器人朝向  
    goalTheta=toDegree(atan2(goal(2)-x(2),goal(1)-x(1)));% 目标点方位  
      
    if goalTheta>theta  
        targetTheta=goalTheta-theta; 
    else  
        targetTheta=theta-goalTheta;
    end  
      
    heading=180-targetTheta; % 误差越小，分数越高
end
  
% model [最高速度m/s, 最高角速度rad/s, 加速度m/ss, 旋转加速度rad/ss, 速度分辨率m/s, 转速分辨率rad/s]  
function Vr=CalcDynamicWindow(x,model)  
    % 计算动态窗口：也就是当前能达到的速度范围
    global dt;  
    % 1. 车辆物理极限
    Vs=[0 model(1) -model(2) model(2)];  
      
    % x = [x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]  
    % 2. 考虑加速度后的实际可达范围
    Vd=[x(4)-model(3)*dt x(4)+model(3)*dt x(5)-model(4)*dt x(5)+model(4)*dt];  
      
    % 3. 取交集
    Vtmp=[Vs;Vd];  
    Vr=[max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];  
end
  
function x = f(x, u)  
    % 运动学模型：根据速度推算下一刻位置
    global dt;  
    F = [1 0 0 0 0;
         0 1 0 0 0; 
         0 0 1 0 0;
         0 0 0 0 0;
         0 0 0 0 0];  
    B = [dt*cos(x(3)) 0; dt*sin(x(3)) 0; 0 dt; 1 0; 0 1];  
    x= F*x+B*u;  
end
  
function radian = toRadian(degree)    
    radian = degree/180*pi;  
end
  
function degree = toDegree(radian)    
    degree = radian/pi*180; 
end

