function [fs, gen] = fourier_trajectory(gen,ident,traj)

%% Options
fs.Nl = 5; 
fs.w = 0.1 ;%rad/s

fs.ts = 0.01;
fs.period = 28 ;%s.
fs.N = fs.period/fs.ts;

fs.time = linspace(0,fs.period-fs.ts,fs.N);

%% Fourier Expansion

fs.a=sym('a%d_%d',[gen.dof, fs.Nl]);
fs.b=sym('b%d_%d',[gen.dof, fs.Nl]);
fs.q0 = sym('q0_%d', [gen.dof, 1]);
syms t real

fs.qi = fs.q0;
fs.qdi = sym(zeros(gen.dof,1));
fs.qddi = sym(zeros(gen.dof,1));
%Nl = length(fs.a);

for j = 1:gen.dof
for i =1:fs.Nl
    fs.qi(j) = fs.qi(j)+fs.a(j,i)/(fs.w*i)*sin(fs.w*i*t)-fs.b(j,i)/(fs.w*i)*cos(fs.w*i*t);
    fs.qdi(j) = fs.qdi(j)+fs.a(j,i)*cos(fs.w*i*t)+fs.b(j,i)*sin(fs.w*i*t);
    fs.qddi(j) = fs.qddi(j)-fs.a(j,i)*(fs.w*i)*sin(fs.w*i*t)+fs.b(j,i)*(fs.w*i)*cos(fs.w*i*t);    
end
end
%% Filter torque
T=linspace(1,length(ident.tau),length(ident.tau));

windowSize = ident.window; 
b = (1/windowSize)*ones(1,windowSize);
a = ident.a;
x1= ident.tau(:,1).';
%zi = zeros(1,max(length(a),length(b))-1);

fs.tau_filter(:,1) = filter(b,a,x1)';
figure()
plot(T,ident.tau(:,1).',T,fs.tau_filter(:,1).');


%% Calculate Cost Function
fs.cov = cov(ident.tau);

%Faster implementation not done
%f = matlabFunction(fs.qi(1))
%f(fs.a(1,1),fs.a(1,2),fs.a(1,3),fs.a(1,4),fs.a(1,5),fs.b(1,1),fs.b(1,2),fs.b(1,3),fs.b(1,4),fs.b(1,5),fs.q0(1),1);

for i =1:fs.N
    
    tic
    A=sym([]);
    for j =1 :gen.dof
        q(j) = fs.qi(j); 
        qd(j) = fs.qdi(j);
        qdd(j) = fs.qddi(j);
        
        A = [A,q(j), qd(j),qdd(j)];
    end
    A = subs(A,t, fs.time(i));
    toc
    
   fs.F((i-1)*gen.dof+1:(i-1)*gen.dof+gen.dof,:) = gen.condfun(A(1),A(4),A(7),A(2),A(5),A(8),A(3),A(6),A(9));
    
end

fs.cost = fs.cov^-0.5*ident.W;




end