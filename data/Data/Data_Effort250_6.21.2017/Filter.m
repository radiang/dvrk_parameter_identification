clear all
T=csvread('data_torque.csv');
scale=0.5;

hz=250;
Time=linspace(0,length(T)-1,length(T))*1/hz;

Fs=250;
%Fnorm = 300/(Fs/2);  % Normalized frequency
Fnorm=0.005;
df = designfilt('lowpassfir','FilterOrder',70,'CutoffFrequency',Fnorm);

%grpdelay(df,2048,Fs) ;  % plot group delay
D = mean(grpdelay(df)) ;% filter delay in samples

Tf1 = filter(df,[T(:,1); zeros(D,1)]); % Append D zeros to the input data
Tf1 = Tf1(D+1:end);                  % Shift data to compensate for delay
Tf2 = filter(df,[T(:,2); zeros(D,1)]); % Append D zeros to the input data
Tf2 = Tf2(D+1:end);  
Tf3 = filter(df,[T(:,3); zeros(D,1)]); % Append D zeros to the input data
Tf3 = Tf3(D+1:end);  

figure(1)
plot(Time,T(:,1),Time,Tf1,'r','linewidth',1.5);
title('Filtered Waveforms');
xlabel('Time (s)')
legend('Original Noisy Signal','Filtered Signal');
grid on
axis tight

figure(2)
plot(Time,T(:,2),Time,Tf2,'r','linewidth',1.5);
title('Filtered Waveforms');
xlabel('Time (s)')
legend('Original Noisy Signal','Filtered Signal');
grid on
axis tight

figure(3)
plot(Time,T(:,3),Time,Tf3,'r','linewidth',1.5);
title('Filtered Waveforms');
xlabel('Time (s)')
legend('Original Noisy Signal','Filtered Signal');
grid on
axis tight

csvwrite('T1.csv',Tf1');
csvwrite('T2.csv',Tf2');
csvwrite('T3.csv',Tf3');
