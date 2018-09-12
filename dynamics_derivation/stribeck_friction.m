close all
rng = 0.05;
v = linspace(-rng, rng, 100);

Fs = 1.1510;
Fc = 0.1597;
vs = 0.0055;
delta_s = 0.6;
sigma = 0.934;
 
F =  [Fc + (Fs - Fc)*exp(-abs((v ./vs)).^delta_s)].*sign(v)+sigma*v;

figure()
plot(v,F)

         
 for j = 1:length(v)
 
     qd3 = v(j);
    Fr3(j)  =  (885256813805337*qd3)/281474976710656 + (161991112331491*exp(-((2000*abs(qd3))/11)^(3/5))*sign(qd3))/140737488355328 - (5753588735520961*sign(qd3)*(exp(-((2000*abs(qd3))/11)^(3/5)) - 1))/36028797018963968 - 103298595163513/562949953421312;
    
     Fr3(j) =  Fr3(j);
 end
 
 figure()
 plot(v,Fr3)
 
 
   