function [fs, gen] = fourier_trajectory_run(gen,traj)

%% Options
fs.Nl = 5; 
fs.w = 0.14*2*pi() ;%rad/s

fs.ts = 0.02;
fs.period = 30;%s.

%% 
fs.disc_num = fs.period/fs.ts;
fs.time = linspace(0,fs.period-fs.ts,fs.disc_num);


%% Filter torque
% T=linspace(1,length(ident.tau),length(ident.tau));
% 
% windowSize = ident.window; 
% b = (1/windowSize)*ones(1,windowSize);
% a = ident.a;
% x1= ident.tau(:,1).';
% %zi = zeros(1,max(length(a),length(b))-1);
% 
% fs.tau_filter(:,1) = filter(b,a,ident.tau(:,1))';
% fs.tau_filter(:,2) = filter(b,a,ident.tau(:,2))';
% fs.tau_filter(:,3) = filter(b,a,ident.tau(:,3))';
% 
% 
% % figure()
% % subplot(3,1,1)
% % plot(T,ident.tau(:,1).',T,fs.tau_filter(:,1).');
% % hold on
% % subplot(3,1,2)
% % plot(T,ident.tau(:,2).',T,fs.tau_filter(:,2).');
% % subplot(3,1,3)
% % plot(T,ident.tau(:,3).',T,fs.tau_filter(:,3).');
% 
% fs.tau_noise = ident.tau(:,:)-fs.tau_filter(:,:);
% fs.scale_noise = 1./std(fs.tau_noise);
% 
% %fs.cov = cov(ident.tau);
% %fs.cov=cov(ident.tau(1:fs.disc_num*gen.dof,:).');
% fs.cov = 1;
% 
% four.scale= 1./max(abs(ident.tau));
% fs.scale = four.scale;
%% Cost Function Variables
%four.cov = fs.cov;
four.dof=gen.dof;
%four.scale_noise=fs.scale_noise;

four.N = fs.Nl;
four.time = fs.time;
four.disc_num =fs.disc_num;
four.size = length(gen.Par2);

four.fun=gen.condfun;

four.w = fs.w; %rad/s

four.Ys2 = gen.Ys2;

%% Initial Condition

%z0 = 0.1*ones(1,(2*fs.Nl+1)*(gen.dof));
%z0((2*fs.Nl+1)*(gen.dof-1)+1:(2*fs.Nl+1)*(gen.dof))= z0((2*fs.Nl+1)*(gen.dof-1)+1:(2*fs.Nl+1)*(gen.dof))/80;

%z0 = 0.0001*ones(1,(2*fs.Nl+1)*(gen.dof));
%z0(33)=0.1;


% From Paper

test.v(:,:) = [0.05 -0.29 0.48 0.55 0.65 0.19 -0.4 -0.18 0.63 -0.46 -0.29;
                0.03 0.29 -0.23 0.32 0.82 0.09 -0.08 0.05 -0.02 0.65 0.11;
                -0.07 0.4 0.45 0.40 -0.03 -0.49 0.32 -0.26 -0.63 0.06 0.04];
            
scale = [0.98, 1, 0.5].';
test.v(:,:) = abs(test.v(:,:)).*scale;
   

z0 = reshape(test.v.',1,[]);



%% Make bounds
for j = 1:gen.dof
    
for i = 1:fs.Nl
    a = traj.limit_pos(j)*fs.w*i;
    b = traj.limit_vel(j);
    
    x = min(abs([a,b]));
    %x = 100;
    
lb_arr(j,i)       = -x;
lb_arr(j,i+fs.Nl) = -x;
ub_arr(j,i)       =  x;
ub_arr(j,i+fs.Nl) =  x;

% Ae(j,i)=1/(fs.w*i);
% Ae(j,i+fs.Nl) = 1/(fs.w*i);
% Ae(j,1+2*fs.Nl) = 1;

end
lb_arr(j,1+fs.Nl*2)   = -traj.limit_pos(j);
ub_arr(j,1+fs.Nl*2)   = traj.limit_pos(j);

if j==3
    lb_arr(j,1+fs.Nl*2)   = 0.005;
end
% b(j)=traj.limit_pos(j);
% b(j+gen.dof)=-traj.limit_pos(j);

end


lb = reshape(lb_arr.',1,[]);
ub = reshape(ub_arr.',1,[]);

A=[];
b=[];
Ae=[];
be=[];

%% Constraints
%nonloncon = @(z) max_fourier(z,four,traj.limit_pos(1:gen.dof),traj.limit_vel(1:gen.dof));
 n = 2*four.N+1;
% 
% %A Matrix
for k = 1:gen.dof 
for j = 1:four.disc_num
    D(j+(k-1)*four.disc_num,n*k) = 1;
    E(j+(k-1)*four.disc_num,n*k) = 0;
    
    d(j+(k-1)*four.disc_num,1) = traj.limit_pos(k);
    e(j+(k-1)*four.disc_num,1) = traj.limit_vel(k);
    
    for i =1:four.N
        D(j+(k-1)*four.disc_num,i+(k-1)*n) = 1/(four.w*i)*sin(four.w*i*four.time(j));
        D(j+(k-1)*four.disc_num,i+four.N+(k-1)*n) = -1/(four.w*i)*cos(four.w*i*four.time(j));
        
        E(j+(k-1)*four.disc_num,i+(k-1)*n) = cos(four.w*i*four.time(j));
        E(j+(k-1)*four.disc_num,i+four.N+(k-1)*n) = sin(four.w*i*four.time(j));
        
    end
end 
end

%b coefficients
A = [D;E];
b = [d;e];

w = b;
w(w==traj.limit_pos(3))=-0.005;

A = [A ; -A];
b = [b;w];
%% Run Optimization
fun = @(z) fourier_function(z,four);
options = optimoptions(@fmincon,'Display','iter','Algorithm','active-set','MaxIterations',800,'MaxFunctionEvaluations',25000);
[fs.vars, fs.opt_cond]=fmincon(fun,z0,A,b,Ae,be,lb,ub,[],options);


%% Make Cost Function
%fs.F=simplify(fs.F);





end