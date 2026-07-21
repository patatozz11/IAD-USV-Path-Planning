function [evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam)
% Vr = [min speed, max speed, min yaw rate, max yaw rate] after one time step.
% model = [max speed, max yaw rate, acceleration, yaw acceleration, speed resolution, yaw-rate resolution].
evalDB=[];
trajDB=[];
for vt=Vr(1):model(5):Vr(2) % min speed : speed resolution : max speed
    for ot=Vr(3):model(6):Vr(4) % min yaw rate : yaw-rate resolution : max yaw rate
        % Predict the trajectory for each candidate control [vt, ot].
        [xt,traj]=GenerateTrajectory(x,vt,ot,evalParam(4));   % Predicted terminal state and trajectory.
        % Calculate evaluation terms.
        heading=CalcHeadingEval(xt,goal); % Heading score at the predicted terminal state.
        dist=CalcDistEval(xt,ob,R);       % Minimum obstacle distance score with saturation.
        vel=abs(vt); % Use the absolute linear velocity.
        stopDist=CalcBreakingDist(vel,model); % Braking distance for safety checking.

        if dist>stopDist % 
            % Record only safe trajectories.
            evalDB=[evalDB;[vt ot heading dist vel]]; % [speed, yaw rate, heading score, distance score, velocity score]
            trajDB=[trajDB;traj]; % Store the full predicted trajectory.
        end
    end
end
end

function stopDist=CalcBreakingDist(vel,model)  
    % Calculate braking distance from the kinematic model.
    global dt;  
    stopDist=0;  
    while vel>0  
        stopDist=stopDist+vel*dt; 
        vel=vel-model(3)*dt; % model(3) is linear acceleration.
    end  
end

function dist=CalcDistEval(x,ob,R)  
    % Obstacle distance evaluation: larger distance is better.
    dist=100;  
    for io=1:length(ob(:,1))  
        disttmp=norm(ob(io,:)-x(1:2)')-R;
        if dist>disttmp% Keep the nearest obstacle distance.
            dist=disttmp;  
        end  
    end  
    % Saturate the maximum value to avoid excessive weight for far obstacles.
    if dist>=R  
        dist=R;  
    end  
end

function heading=CalcHeadingEval(x,goal)  
    % Heading evaluation: better score when the heading points closer to the goal.
    theta=toDegree(x(3));                                % Current heading.
    goalTheta=toDegree(atan2(goal(2)-x(2),goal(1)-x(1)));% Bearing to the goal.
      
    if goalTheta>theta  
        targetTheta=goalTheta-theta; 
    else  
        targetTheta=theta-goalTheta;
    end  
    heading=180-targetTheta; % Smaller heading error gives a higher score.
end

function degree = toDegree(radian)    
    degree = radian/pi*180; 
end
