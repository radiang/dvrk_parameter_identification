%Open Calibrated_PSM_Dynamics_6DOF_Final.mat
function [Mt, Nu, can] = seperate_f(Ys2,Par2,Qdd)

%% Start Seperation of Parameters
can = sym('can%d', [1 length(Par2)],'real'); %Lumped parameters expressed as a single variable
can_Y = Ys2; %Lumped 6x31 observation matrix

Mult = can_Y*can'; %Equals the manipulator equation

%Seperate the Inertia Matrix with Lumped Parameters
[Mt, Nu]=equationsToMatrix(Mult == 0, Qdd); %Seperate the Inertia matrix to lumped parameters

symvar(Nu); 

%save('6dof_lumped_final_equations.mat', 'Mt','Nu','can_Y','can','Par2')

%n = Mult - Mt*Qdd; %Resulting Coriolis and Gravity Terms for Feedforward Compensation

end
