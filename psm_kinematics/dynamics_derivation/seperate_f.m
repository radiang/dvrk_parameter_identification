%Open Calibrated_PSM_Dynamics_6DOF_Final.mat
function [Mt, Nu, Cxqd, G, Fr] = seperate_f(gen,dyn)
syms q1 q2 q3 qd1 qd2 qd3 real 

%% Start Seperation of Parameters
%can = sym('can%d', [1 length(Par2)],'real'); %Lumped parameters expressed as a single variable
can = gen.sdp_par2;
can_Y = gen.Ys2; %Lumped 6x32observation matrix

%Seperate the Friction Matrix
syms Fco_3 Fso_3 Fc_3
m = [];
array = [dyn.Fs(1:gen.dof).', dyn.Fv(1:gen.dof).', dyn.Ke(1:gen.dof-1).', Fc_3, Fco_3+Fso_3];

for i = 1:length(array)
m(end+1)=find(gen.Par2==array(i));
end


m = sort(m,'descend');
Fr = sym(zeros(gen.dof,1));
for i = 1:length(m)
    Fr(:,1) = Fr(:,1)+can_Y(:,m(i)).*can(m(i));
    
    can_Y(:,m(i))=[];
    can(m(i))=[];
end

Mult = can_Y*can; %Equals the manipulator equation

%Seperate the Inertia Matrix with Lumped Parameters
[Mt, Nu]=equationsToMatrix(Mult == 0, gen.Qdd(1:gen.dof)); %Seperate the Inertia matrix to lumped parameters

Nu = -Nu;

% get Coriolis Term
G= subs(Mult,[gen.qdd(1:gen.dof) gen.qd(1:gen.dof)], [0 0 0 0 0 0]);

remain = Mult-Mt*gen.Qdd(1:gen.dof)-G;
remain = collect(remain,[gen.qdd(1:gen.dof), gen.qd(1:gen.dof)]);
remain = simplify(remain);

Cxqd = remain;
%C=[diff(remain,gen.Qd(1)),diff(remain,gen.Qd(2)),diff(remain,gen.Qd(3))];

symvar(G); 

x = 0;
%save('6dof_lumped_final_equations.mat', 'Mt','Nu','can_Y','can','Par2')

%n = Mult - Mt*Qdd; %Resulting Coriolis and Gravity Terms for Feedforward Compensation

end
