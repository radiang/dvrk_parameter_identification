function [fs,gen]=check_fourier(gen,ident,traj,fs);

test.v= reshape(fs.vars,[],gen.dof).';
test.dof = gen.dof;
test.time = fs.time;
test.N=fs.Nl;
test.w = fs.w;
%% Input
[qi, qdi,qddi]= fourier_f(test);


%% Condition number of regressor
for i=1:length(qi(1,:)) 
    W(1+(i-1)*gen.dof:gen.dof+(i-1)*gen.dof,:)=gen.condfun(qi(1,i),qi(2,i),qi(3,i),qdi(1,i),qdi(2,i),qdi(3,i),qddi(1,i),qddi(2,i),qddi(3,i));
    
  
 %W(1+(i-1)*dof_num:dof_num+(i-1)*dof_num,:)=subs(Ys2, transpose([Q(1:dof_num); Qd(1:dof_num) ;Qdd(1:dof_num)]),[q(i,1:3), qdf(i,1:3), acc(i,1:3)]);
end

fs.cond=cond(W);
%% plot
figure()
plot(test.time,qi(1,:),test.time,qi(2,:),test.time,qi(3,:));
legend('1','2','3')

end