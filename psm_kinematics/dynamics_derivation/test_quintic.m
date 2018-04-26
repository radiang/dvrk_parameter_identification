clear all 

Q = [0, 0.2, -0.2, 0.5];
Qd = [0, 0.2, 0, 0.3];
Qdd = [ 0, 0.3, 0.2, 0];

tf = 2;
ts = 0.1;

[x,v,ac,T]= Trajectory_quintic(Q,Qd,Qdd,tf,ts,0);


plot(T,x,T,v,T,ac);
legend('x','v','acc');
