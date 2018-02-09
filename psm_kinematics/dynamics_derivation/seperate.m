%Open Calibrated_PSM_Dynamics_6DOF_Final.mat
clear all
load('Calibrated_PSM_Dynamics_6DOF_Final1.mat')

%% Start Seperation of Parameters
can = sym('can%d', [1 length(Par2)]);
can_Y = Ys2;

Mult = can_Y*can';

%Seperate the Inertia Matrix with Lumped Parameters
[Mt, Nu]=equationsToMatrix(Mult == 0, Qdd);

symvar(Nu);

save('6dof_lumped_final_equations.mat', 'Mt','Nu','can_Y','can','Par2')

