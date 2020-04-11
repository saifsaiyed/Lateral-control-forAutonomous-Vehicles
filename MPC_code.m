%%
%Generating Reference signals
%Lateral Position
t=linspace(0,1,40);
x=[1 30 60 90 120];
y=[0 3 6];

x_1=(1-t).^2*x(1)+2*t.*(1-t)*x(2)+(t.^2)*x(3);
x_2=(1-t).^2*x(3)+2*t.*(1-t)*x(4)+(t.^2)*x(5);
y_1=(1-t).^2*y(1)+2*t.*(1-t)*y(1)+(t.^2)*y(2);
y_2=(1-t).^2*y(2)+2*t.*(1-t)*y(3)+(t.^2)*y(3);

x_pos=[x_1 x_2(2:end)];
y_pos=[y_1 y_2(2:end)];
time=0:0.1:8;
y_ref=[time;[y_pos(1) y_pos y_pos(end)]]';


%Yaw angle
psy_n_1=-2*(1-t)*y(1)+2*(1-t)*y(1)-2*t*y(1)+2*t*y(2);
psy_d_1=-2*(1-t)*x(1)+2*(1-t)*x(2)-2*t*x(2)+2*t*x(3);
psy_1=atan(psy_n_1./psy_d_1);
psy_n_2=-2*(1-t)*y(2)+2*(1-t)*y(3)-2*t*y(3)+2*t*y(3);
psy_d_2=-2*(1-t)*x(3)+2*(1-t)*x(4)-2*t*x(4)+2*t*x(5);
psy_2=atan(psy_n_2./psy_d_2);

psy=[psy_1 psy_2(2:end)];
psy_ref=[time;[psy(1) psy psy(end)]]';

%Plot
plot(x_pos,y_pos);
ah=gca;
ah.YLim=[0 20];
yyaxis right;
plot(x_pos,psy);
ah.YLim=[0 0.2];

%%
%Defining My Car Model
%[y dy psy dpsy]
val=struct('cf',80000,'cr',96000,'vx',30,'m',2325,'I',4132,'lf',1.43,'lr',1.595);
A=[0 1 0 0; 0 -2*(val.cf+val.cr)/val.m/val.vx 0 -val.vx-(2*(val.cf*val.lf-val.cr*val.lr)/val.m/val.vx);0 0 0 1;0 -(2*(val.cf*val.lf-val.cr*val.lr))/val.I/val.vx 0 -(2*(val.cf*val.lf^2+val.cr*val.lr^2))/val.I/val.vx];
B= [0 2*val.cf/val.m 0 2*val.lf*val.cf/val.I].';
C=[1 0 val.vx 0; 0 0 1 0];
D=[0;0];

rank(obsv(A,C))
rank(ctrb(A,B))



%Defining My Car Model With Tutorial Values
%[y dy psy dpsy]
val=struct('cf',19000,'cr',33000,'vx',15,'m',1575,'I',2875,'lf',1.2,'lr',1.6);
A=[0 1 0 0; 0 -2*(val.cf+val.cr)/val.m/val.vx 0 -val.vx-(2*(val.cf*val.lf-val.cr*val.lr)/val.m/val.vx);0 0 0 1;0 -(2*(val.cf*val.lf-val.cr*val.lr))/val.I/val.vx 0 -(2*(val.cf*val.lf^2+val.cr*val.lr^2))/val.I/val.vx];
B= [0 2*val.cf/val.m 0 2*val.lf*val.cf/val.I].';
C=[1 0 val.vx 0; 0 0 1 0];
D=[0;0];

rank(obsv(A,C))
rank(ctrb(A,B))

        mpc1=mpc(ss(A,B,C,D),0.1);
        getEstimator(mpc1)%Error

        mpc2=mpc(ss(A,B,C,D),0.5);
        getEstimator(mpc2)%No Error

        mpc3=mpc(ss(A,B,C,D),0.9);
        getEstimator(mpc3)%No Error
        
        rank(ctrb(c2d(ss(A,B,C,D),0.1)))
        rank(ctrb(c2d(ss(A,B,C,D),0.5)))
        rank(ctrb(c2d(ss(A,B,C,D),0.9)))

        
        %[dy psy dpsy]
        A=[-2*(val.cf+val.cr)/val.m/val.vx 0 -val.vx-(2*(val.cf*val.lf-val.cr*val.lr)/val.m/val.vx);0 0 1;-(2*(val.cf*val.lf-val.cr*val.lr))/val.I/val.vx 0 -(2*(val.cf*val.lf^2+val.cr*val.lr^2))/val.I/val.vx];
        B= [2*val.cf/val.m 0 2*val.lf*val.cf/val.I].';
        C=[1 val.vx 0];
        D=[0];

        rank(obsv(A,C))
        rank(ctrb(A,B))
        
        car=ss(A,B,C,D)
        carm=ss(car,'minimal') 
        
        
%Defining Tutorial Car Model with changed vectors
%[Y dY psy dpsy]
val=struct('cf',19000,'cr',33000,'vx',15,'m',1575,'I',2875,'lf',1.2,'lr',1.6);
A=[0 1 val.vx 0;0 -2*(val.cf+val.cr)/val.m/val.vx 0 -val.vx-(2*(val.cf*val.lf-val.cr*val.lr)/val.m/val.vx);0 0 0 1;0 -(2*(val.cf*val.lf-val.cr*val.lr))/val.I/val.vx 0 -(2*(val.cf*val.lf^2+val.cr*val.lr^2))/val.I/val.vx];
B=[0 2*val.cf/val.m 0 2*val.lf*val.cf/val.I].';
C=[1 0 0 0;0 0 1 0];
D=[0;0];

rank(obsv(A,C))
rank(ctrb(A,B))



%%
%Backup
rank(obsv(c2d(ss(A,B,C,D),0.1)))
rank(ctrb(c2d(ss(A,B,C,D),0.1)))
rank(obsv(c2d(ss(A,B,C,D),0.5)))
rank(ctrb(c2d(ss(A,B,C,D),0.5)))

%Exact Car Model
val=struct('cf',19000,'cr',33000,'vx',15,'m',1575,'I',2875,'lf',1.2,'lr',1.6);
A=[-2*(val.cf+val.cr)/val.m/val.vx 0 -val.vx-(2*(val.cf*val.lf-val.cr*val.lr)/val.m/val.vx) 0;0 0 1 0;-(2*(val.cf*val.lf-val.cr*val.lr))/val.I/val.vx 0 -(2*(val.cf*val.lf^2+val.cr*val.lr^2))/val.I/val.vx 0;1 val.vx 0 0];
B=[2*val.cf/val.m 0 2*val.lf*val.cf/val.I 0].';
C=[0 0 0 1; 0 1 0 0];
D=[0;0];

pzmap(ss(A,B,C,D));
rank(obsv(A,C))
rank(ctrb(A,B))


x=zeros(80,1);
for i=1:80
    x(i)=y_ref(i+1,2)-y_ref(i,2);
end
x