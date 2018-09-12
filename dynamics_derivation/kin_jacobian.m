
function [ctrl] = kin_jacobian(ctrl) 

%% Kinematic Jacobian 
syms q1 q2 q3 q4 q5 q6 
gen.q   = [q1 q2 q3 q4 q5 q6];

kin.p = sym(pi());

kin.a     = [0       ,                 0,              0,               0];
kin.alpha = [-kin.p/2,          -kin.p/2,       -kin.p/2,               0]; 
kin.d     = [0       ,                 0,              0, 0.0037+gen.q(3)];
kin.tet   = [kin.p/2 , -kin.p/2+gen.q(1),kin.p/2+gen.q(2)        -kin.p/2];


for i=1:length(kin.a)
kin.Ti_i(1:4,1:4,i) = DH(kin.tet(i),kin.a(i),kin.d(i),kin.alpha(i));

end

for i=1:length(kin.a)
    if i==1
        kin.T(1:4,1:4,i) = kin.Ti_i(:,:,i);
    else
    kin.T(1:4,1:4,i)=kin.T(1:4,1:4,i-1)*kin.Ti_i(1:4,1:4,i);
    %T(1:4,1:4,j)=vpa(T(1:4,1:4,j),4);
    end
end


figure()
for i = 1:length(kin.T)
        T_num(:,:,i)=subs(kin.T(:,:,i),gen.q,ctrl.qn);
        
        scatter3(T_num(1,4,i),T_num(2,4,i),T_num(3,4,i));
        marker_id = sprintf('%d',i);
        line(T_num(1,4,i),T_num(2,4,i),T_num(3,4,i));
        text(T_num(1,4,i),T_num(2,4,i),T_num(3,4,i),marker_id);  
        hold on
end

Jac = [diff(kin.T(1:3,end),gen.q(1)),diff(kin.T(1:3,end),gen.q(2)),diff(kin.T(1:3,end),gen.q(3))];

Jac_num = subs(Jac,gen.q,ctrl.qn);

ctrl.kin_Jac = Jac;

disp('Kinematic Model')
%inv_jac = double(inv(Jac_num))*ctrl.vd.'
inv_jac = double(inv(Jac_num))
end

