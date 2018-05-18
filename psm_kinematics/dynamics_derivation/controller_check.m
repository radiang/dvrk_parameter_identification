function [ctrl] = controller_check(gen)

%% 3DOF FORCE Controller 
syms  q1t(t) q2t(t) q3t(t) q4t(t) q5t(t) q6t(t) 

qt = [q1t(t) q2t(t) q3t(t) 0 0 0];
qdt = diff(qt,t);
qddt = diff(qdt,t);

%% Options
debug = 0;

%% Jacobian End Effector and it's Derivative
J_end_t = subs(gen.J_end, symvar(gen.J_end), qt);
ctrl.J_diff_t = diff(J_end_t,t);

%Make 3dof Jacobian 
ctrl.J3 = subs(gen.J_end(1:3,1:3),[gen.q(4), gen.q(5), gen.q(6)],[0 0 0]);

J3_diff = subs(ctrl.J_diff_t(1:6,1:3), [diff(q1t,t),diff(q2t,t),diff(q3t,t)],transpose(gen.Qd(1:3)));
J3_diff = subs(J3_diff, q1t, gen.Q(1));
J3_diff = subs(J3_diff, q2t, gen.Q(2));
ctrl.J3_diff = subs(J3_diff, q3t, gen.Q(3));


%% Check Jacobian in plot
p = pi();
q_n = [0, p/4, .1, 0, 0, 0];

J3_num = subs(J_end_t(1:6,1:3),q1t,q_n(1));
J3_num = subs(J3_num,q2t,q_n(2));
J3_num = subs(J3_num,q3t,q_n(3));

J3_num = double(J3_num);


% for i = 1:length(T)
%         T_num(:,:,i)=subs(T(:,:,i),[q1 q2 q3 q4 q5 q6],q_n);
%         
%         scatter3(T_num(1,4,i),T_num(2,4,i),T_num(3,4,i));
%         line(T_num(1,4,i),T_num(2,4,i),T_num(3,4,i));
%         marker_id = sprintf('%d',i);
%         text(T_num(1,4,i),T_num(2,4,i),T_num(3,4,i),marker_id);     
%         hold on
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%figure()
% T_num(:,:,:)=double(subs(dyn.T(:,:,:),[gen.q(1:6)],q_n));
% a(1,:) = reshape(T_num(1,4,:),1,[]);
% a(2,:) = reshape(T_num(2,4,:),1,[]);
% a(3,:) = reshape(T_num(3,4,:),1,[]);
% scatter3(a(1,:),a(2,:),a(3,:));
% line(a(1,:),a(2,:),a(3,:));
% hold on
% 
% x0 = double(subs(dyn.T(1:3,4,11), gen.q, q_n));
% 
% xv=J3_num*transpose([10 0 0]);
% xvx=xv
% xd = x0+xv(1:3);
% plot3([xd(1),T_num(1,4,11)],[xd(2),T_num(2,4,11)],[xd(3),T_num(3,4,11)],'r');
% hold on
% xv=J3_num*transpose([0 10 0]);
% xvy=xv
% xd = x0+xv(1:3);
% plot3([xd(1),T_num(1,4,11)],[xd(2),T_num(2,4,11)],[xd(3),T_num(3,4,11)],'g');
% xv=J3_num*transpose([0 0 0.5]);
% xd = x0+xv(1:3);
% plot3([xd(1),T_num(1,4,11)],[xd(2),T_num(2,4,11)],[xd(3),T_num(3,4,11)],'b');
% 
% 
% title('Plot transform Frames');
% xlabel('x');
% ylabel('y');
% zlabel('z');
% s=1;
% xlim([-s/2 s])
% ylim([-s/2 s])
% zlim([-s/2 s])


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% for i = 1:length(T_cg)
%         T_cg_num(:,:,i)=subs(T_cg(:,:,i),[q1 q2 q3 q4 q5 q6 lc1 lc2 lc3 lc4 lc6 lc7 lc8 lc9], [q_n lc_n]);
%         
%         scatter3(T_cg_num(1,4,i),T_cg_num(2,4,i),T_cg_num(3,4,i),'*');
%         marker_id = sprintf('cg_%d',i);
%         text(T_cg_num(1,4,i),T_cg_num(2,4,i),T_cg_num(3,4,i),marker_id);
%         hold on
% end

%  for i = 1:length(p_cg)
%          p_cg_num(i,:)=subs(p_cg(i,:),[q l_cg(i,:)], [q_n, 0, 0, .1]);
%          %if(i>9)
%          scatter3(p_cg_num(i,1),p_cg_num(i,2),p_cg_num(i,3),'*');
%          marker_id = sprintf('cg_%d',i);
%          text(p_cg_num(i,1),p_cg_num(i,2),p_cg_num(i,3),marker_id);
%          hold on
%          %end
%  end



%% Seperate Variables to Par2 Force Controller 
[Mt, Nu]=seperate_f(gen.Ys2,gen.ls_par2,gen.Qdd);

%% Make into 3 DOF
ctrl.Mt3 = subs(Mt(1:3,1:3),[transpose(gen.Q(4:6))], [0 0 0]);
ctrl.Nu3 = subs(Nu(1:3,1),[transpose(gen.Q(4:6)),transpose(gen.Qd(4:6))],[0, 0, 0, 0, 0, 0]);
%% Generate Ccode
 stringname = strcat('ccode/',gen.filename,'/',gen.fourfilename,'_J_ccode.c');
 ccode(ctrl.J3,'File',stringname,'Comments','V1.2');
% 
 stringname = strcat('ccode/',gen.filename,'/',gen.fourfilename,'_Jd_ccode.c');
 ccode(ctrl.J3_diff,'File',stringname,'Comments','V1.2');
% 
 stringname = strcat('ccode/',gen.filename,'/',gen.fourfilename,'_Mt_ccode.c');
 ccode(ctrl.Mt3,'File',stringname,'Comments','V1.2');
% 
 stringname = strcat('ccode/',gen.filename,'/',gen.fourfilename,'_Nu_ccode.c');
 ccode(ctrl.Nu3,'File',stringname,'Comments','V1.2');
% 
 stringname = strcat('ccode/',gen.filename,'/',gen.fourfilename,'_Ys2_ccode.c');
 ccode(gen.Ys2,'File',stringname,'Comments','V1.2');



end
