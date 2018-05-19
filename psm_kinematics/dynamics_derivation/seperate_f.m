%Open Calibrated_PSM_Dynamics_6DOF_Final.mat
function [Mt, Nu, C, G] = seperate_f(Ys2,Par2,Qdd,Qd)
syms q1 q2 q3 qd1 qd2 qd3 real
%% Start Seperation of Parameters
%can = sym('can%d', [1 length(Par2)],'real'); %Lumped parameters expressed as a single variable
can = Par2;
can_Y = Ys2; %Lumped 6x32observation matrix

Mult = can_Y*can; %Equals the manipulator equation

%Seperate the Inertia Matrix with Lumped Parameters
[Mt, Nu]=equationsToMatrix(Mult == 0, Qdd); %Seperate the Inertia matrix to lumped parameters


 C = [diff(Nu,Qd(1)), diff(Nu,Qd(2)), diff(Nu,Qd(3))];
 G = Nu - C*Qd(1:3);
 
symvar(Nu); 

%save('6dof_lumped_final_equations.mat', 'Mt','Nu','can_Y','can','Par2')

%n = Mult - Mt*Qdd; %Resulting Coriolis and Gravity Terms for Feedforward Compensation

end
