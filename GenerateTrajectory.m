function [x,traj]=GenerateTrajectory(x,vt,ot,evaldt)
% Trajectory generation function.
% evaldt is the forward simulation time; vt and ot are the sampled linear and angular velocities.
global dt;
time=0;
u=[vt;ot];% Control input.
traj=x;% Predicted trajectory.
while time<=evaldt % For evaldt = 3 and dt = 0.1, this produces 31 samples.
    time=time+dt;% Advance time by one step.
    x=f(x,u);% Motion update after one time step.
    traj=[traj x];% Store all predicted states along the trajectory.
end
end
