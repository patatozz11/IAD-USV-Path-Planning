function [u,trajDB]=DynamicWindowApproach(x,model,goal,evalParam,ob,R)
Vr=CalcDynamicWindow(x,model);  % Calculate the admissible velocity window.
[evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam);

if isempty(evalDB)   % Check whether any candidate trajectory is feasible.
    disp('no path to goal!!');
    u=[0;0];return;
end

evalDB=NormalizeEval(evalDB);   % Normalize evaluation terms.

feval=[]; % Final weighted score.
for id=1:length(evalDB(:,2))
    feval=[feval;evalParam(1:3)*evalDB(id,3:5)'];
end
evalDB=[evalDB feval];
 
[~,ind]=max(feval);% Select the candidate with the maximum score.
u=evalDB(ind,1:2)';% 
end

function Vr=CalcDynamicWindow(x,model)
global dt;
Vs=[0 model(1) -model(2) model(2)];
Vd=[x(4)-model(3)*dt x(4)+model(3)*dt x(5)-model(4)*dt x(5)+model(4)*dt];
Vtmp=[Vs;Vd];
Vr=[max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];
% Vr = [min speed, max speed, min yaw rate, max yaw rate] after one time step.
end

function EvalDB=NormalizeEval(EvalDB)
% Normalize evaluation terms.
% EvalDB = [predicted speed, yaw rate, heading score, obstacle distance score, velocity score].
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

