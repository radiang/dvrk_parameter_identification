%% Test Matrices
clear all
qn  = [0.2, 0.1, 0];
qdn = [0, 0, -0.02];
test_num = 6;
dof_num = 3;
disp('-------------------------start----------------------------')

for j = 1:test_num

if (j==1)
    string = sprintf('');
else    
    string =sprintf('%d',j);
end
loadname = strcat('data/test_3dof_svd/fourier_test',string,'_results.mat' );
load(loadname);

nM(:,:,j) = double(subs(ctrl.Mt3, gen.q(1:3), qn));
nN(:,j) = double(subs(ctrl.Nu3, [gen.q(1:3), gen.qd(1:3)], [qn, qdn]));

nC(:,j) = double(subs(ctrl.C, [gen.q(1:3), gen.qd(1:3)], [qn, qdn]));
nF(:,j) = double(subs(ctrl.Fr, [gen.q(1:3), gen.qd(1:3)], [qn, qdn]));
nG(:,j) = double(subs(ctrl.G, gen.q(1:3),qn));

all(:,j) = nC(:,j)+nF(:,j)+nG(:,j);
all_F(:,j) = nF(:,j)+nN(:,j);

end 

all
std_all = std(all.').'
mean_all = mean(all.').'

n_std_G = std(nG.'./range(nG.'))
n_std_F = std(nF.'./range(nF.'))

for k = 1:dof_num
    [c(k) ind(k)] = min(abs(mean_all(k)-all(k,:)));
end 
