close all
rng = 0.05;
v = linspace(-rng, rng, 100);
% 
% Fs = 0.36;
% Fc = 0.27;
% vs = 0.007;
% delta_s = 0.5;
% sigma = 7.7;

Fs = 1.1510;
Fc = 0.1597;
vs = 0.0055;
delta_s = 0.6;
sigma = 0.934;
 
%syms Fc Fs delta_s v sigma vs
F =  [Fc + (Fs - Fc)*exp(-abs((v ./vs)).^delta_s)].*sign(v)+sigma*v;

figure()
plot(v,F)

         
 for j = 1:length(v)
 
     qd3 = v(j);
     %(4206264159485825*qd3)/4503599627370496 + (6812885941224007*exp(-((2000*abs(qd3))/11)^(3/5))*sign(qd3))/19342813113834066795298816 - (5293840339642899*sign(qd3)*(exp(-((2000*abs(qd3))/11)^(3/5)) - 1))/9007199254740992 + 6289463066791451/72057594037927936;
    Fr3(j)  =  (885256813805337*qd3)/281474976710656 + (161991112331491*exp(-((2000*abs(qd3))/11)^(3/5))*sign(qd3))/140737488355328 - (5753588735520961*sign(qd3)*(exp(-((2000*abs(qd3))/11)^(3/5)) - 1))/36028797018963968 - 103298595163513/562949953421312;
    
     Fr3(j) =  Fr3(j);
 end
 
 figure()
 plot(v,Fr3)
 
 

 
%  par = [Fc, Fs, sigma];
% [A b] = equationsToMatrix(F == 0, [Fc, Fs, sigma]);

     
% syms Fc_o Fs_o
% 
% m = find(par == 'Fs');
% A = [ A(1:m), 1 , A(m+1:end)];
% par = [par(1:m), Fc_o, par((m+1):end)];




%An = subs(A, symvar(A), [0.5, 0.1, 0.007]);

% B = [zeros(1,3); An];
% 
% rank(B)

%plot(v,F);


%  Fr1  =                                                                                                               (1758741158185597*qd1)/18014398509481984 - (7606327146036341*q1)/2251799813685248 + (4753537583444945*sign(qd1))/72057594037927936;
%  Fr2  =      