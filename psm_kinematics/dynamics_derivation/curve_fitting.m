% BCD parameter estimation f
function friction_fitting(gen,ctrl)

load('BCD_front (1).mat');
b_init = 13;
c_init = 1.5;
d_init = 1000;

b_init1 = 13;
c_init1 = 1.5;
d_init1 = 1000;

for i = 1:35
    %Paramter Estimation for Front Wheel
    xdata = (S(i).Slip_Favg(3:end))'; 
    ydata = (S(i).F_Favg(2,3:end))';
    f = fittype('d*sin(c*atan(b*x))'); 
    [fit1,gof,fitinfo] = fit(xdata,ydata,f,'StartPoint',[b_init c_init d_init]);
    b_init = fit1.b;
    c_init = fit1.c;
    d_init = fit1.d;
    

    % Parameter Estimation for rear wheel 
    xdata1 = (S(i).Slip_Ravg(3:end))'; 
    ydata1 = (S(i).F_Ravg(2,3:end))';
    f2 = fittype('dr*sin(cr*atan(br*x))'); 
    [fit2,gof2,fitinfo2] = fit(xdata1,ydata1,f2,'StartPoint',[b_init1 c_init1 d_init1]);
    b_init1 = fit2.br;
    c_init1 = fit2.cr;
    d_init1 = fit2.dr;
    
       
end

Bf = fit1.b
Cf = fit1.c
Df = fit1.d;

Br = fit1.b
Cr = fit1.c
Dr = fit1.d

x = (S(37).Slip_Favg(3:end))';
y = Df*sin(Cf*atan(Bf*x));

y1 = (S(37).F_Favg(2,3:end))';

xr = (S(37).Slip_Ravg(3:end))';
yr = Dr*sin(Cr*atan(Br*xr));

y1r = (S(37).F_Ravg(2,3:end))';

figure;
scatter(x,y);
hold on
scatter(x,y1);
legend('Estimated Ffy ', 'Measured Ffy');
xlabel('Front tire slip angle');
ylabel('Lateral Force on Front Tire');

figure;
scatter(xr,yr);
hold on
scatter(xr,y1r);
legend('Estimated Fry ', 'Measured Ffy');
xlabel('Rear tire slip angle');
ylabel('Lateral Force on Rear Tire');


end


% fit1 = 
% 
%      General model:
%      fit1(x) = d*sin(c*atan(b*x))
%      Coefficients (with 95% confidence bounds):
%        b =       10.43  (10.28, 10.58)
%        c =       1.337  (1.329, 1.346)
%        d =        1376  (1374, 1379)