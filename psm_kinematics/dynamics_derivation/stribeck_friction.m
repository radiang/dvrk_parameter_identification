
% rng = 0.05;
% v = linspace(-rng, rng, 100);
% 
% Fs = 0.36;
% Fc = 0.27;
% vs = 0.007;
% delta_s = 0.5;
% sigma = 7.7;
 
syms Fc Fs delta_s v sigma vs

F =  [Fc + (Fs - Fc)*exp(-abs((v ./vs)).^delta_s)].*sign(v) + sigma * v;


par = [Fc, Fs, sigma];
[A b] = equationsToMatrix(F == 0, [Fc, Fs, sigma]);


% syms Fc_o Fs_o
% 
% m = find(par == 'Fs');
% A = [ A(1:m), 1 , A(m+1:end)];
% par = [par(1:m), Fc_o, par((m+1):end)];




An = subs(A, symvar(A), [0.5, 0.1, 0.007]);

% B = [zeros(1,3); An];
% 
% rank(B)

%plot(v,F);