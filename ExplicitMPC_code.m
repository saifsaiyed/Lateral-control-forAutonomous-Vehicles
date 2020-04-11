%%
%Defining My Car Model
%[Y dY psy dpsy]
val=struct('cf',80000,'cr',96000,'vx',30,'m',2325,'I',4132,'lf',1.43,'lr',1.595);
A=[0 1 val.vx 0;0 -2*(val.cf+val.cr)/val.m/val.vx 0 -val.vx-(2*(val.cf*val.lf-val.cr*val.lr)/val.m/val.vx);0 0 0 1;0 -(2*(val.cf*val.lf-val.cr*val.lr))/val.I/val.vx 0 -(2*(val.cf*val.lf^2+val.cr*val.lr^2))/val.I/val.vx];
B=[0 2*val.cf/val.m 0 2*val.lf*val.cf/val.I].';
C=[1 0 0 0;0 0 1 0];
D=[0;0];

%%
%Explicit MPC 2
range = generateExplicitRange(mpc1);

% range.State.Min= [min(states.Data(:,1)) min(states.Data(:,2)) min(states.Data(:,3)) min(states.Data(:,4)) -1];
% range.State.Max= [max(states.Data(:,1)) max(states.Data(:,2)) max(states.Data(:,3)) max(states.Data(:,4)) 1];
range.State.Min= [-6 -10 -0.5236 -0.1745 -0.1]; %Y = 6m max possible value, vy = -10 ms-2
range.State.Max= [6 10 0.5236 0.1745 0.1];      %Yaw = 30 deg maximum, yaw rate = 10 deg/sec

range.Reference.Min= [min(y_ref2(:,2)) min(psy_ref2(:,2))]; %for a fixed maneuvering scenerio
range.Reference.Max= [max(y_ref2(:,2)) max(psy_ref2(:,2))]; %for a fixed maneuvering scenerio

range.Reference.Min= [-4.5 -0.5236]; %going in left lane (4 m lane assumed+0.5m clearnece), 30 deg as maximum yaw for maximum speed
range.Reference.Max= [4.5 0.5236];   %going in right lane (4 m lane assumed+0.5m clearnece), 30 deg as maximum yaw for maximum speed

range.MeasuredDisturbance.Min=[]; % No disturbance assumed
range.MeasuredDisturbance.Max=[]; % No disturbance assumed

range.ManipulatedVariable.Min=-0.8727; % -50 deg, -35 deg is limit in mpc1 but thats soft constraint 
range.ManipulatedVariable.Max=0.8727;  %  50 deg,  35 deg is limit in mpc1 but thats soft constraint

empc2=generateExplicitMPC(mpc1,range);

%Visualization
close;
plotParams = generatePlotParameters(empc2);
plotParams.State.Index=[2 4 5];
plotSection(empc2,plotParams);
ah=gca;
ah.XLabel.String='Absolute position of vehicle, Y (m)';
ah.YLabel.String='Yaw angle of vehicle, Psy (rad)';
hold on;
X=-states.Data(:,1);
Y=states.Data(:,3);
plot(X,Y)
hold off;

empc1.PiecewiseAffineSolution.H
empc1.PiecewiseAffineSolution.K

%Reduction
empc2reduced = simplify(empc2,'radius',0.04);
plotParams = generatePlotParameters(empc2reduced);
plotParams.State.Index=[2 4 5];
plotSection(empc2reduced,plotParams);


%%
%Backup
%Explicit MPC
range = generateExplicitRange(mpc1);
range.State.Min(:)=-10;
range.State.Max(:)=10;
range.Reference.Min(:)=-10;
range.Reference.Max(:)=10;
range.MeasuredDisturbance.Min=[];
range.MeasuredDisturbance.Max=[];
range.ManipulatedVariable.Min=-10;
range.ManipulatedVariable.Max=10;

empc1=generateExplicitMPC(mpc1,range);

%Visualization
plotParams = generatePlotParameters(empc1);
plotParams.State.Index=[2 4 5];
plotSection(empc1,plotParams);
