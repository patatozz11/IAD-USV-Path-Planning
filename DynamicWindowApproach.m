function [u,trajDB]=DynamicWindowApproach(x,model,goal,evalParam,ob,R)
Vr=CalcDynamicWindow(x,model);  %计算可行速度区间
[evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam);

if isempty(evalDB)   %  isempty 判断是否为空
    disp('no path to goal!!');
    u=[0;0];return;
end

evalDB=NormalizeEval(evalDB);   % 各评价函数正则化

feval=[]; % 最终评价函数的计算
for id=1:length(evalDB(:,2))
    feval=[feval;evalParam(1:3)*evalDB(id,3:5)'];
end
evalDB=[evalDB feval];
 
[~,ind]=max(feval);% 最优评价函数
u=evalDB(ind,1:2)';% 
end

function Vr=CalcDynamicWindow(x,model)
global dt;
Vs=[0 model(1) -model(2) model(2)];
Vd=[x(4)-model(3)*dt x(4)+model(3)*dt x(5)-model(4)*dt x(5)+model(4)*dt];
Vtmp=[Vs;Vd];
Vr=[max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];
%  Vr=[ 0.1s后最低速度 0.1s后最高速度 0.1s后最低角速度 0.1s后最高角速度 ]
end

function EvalDB=NormalizeEval(EvalDB)
% 评价函数正则化
% EvalDB=[ 预测的速度 转速 夹角 障碍物距离 速度值  ]
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

