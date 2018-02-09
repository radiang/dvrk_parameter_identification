clear all
T=csvread('data_torque.csv');
scale=0.5;

Fs=250;
%Fnorm = 300/(Fs/2);  % Normalized frequency
Fnorm=0.005;
df = designfilt('lowpassfir','FilterOrder',70,'CutoffFrequency',Fnorm);

%grpdelay(df,2048,Fs) ;  % plot group delay
D = mean(grpdelay(df)) ;% filter delay in samples

Tf1 = filter(df,[T(:,2); zeros(D,1)]); % Append D zeros to the input data
Tf1 = Tf1(D+1:end);                  % Shift data to compensate for delay
Tf2 = filter(df,[T(:,3); zeros(D,1)]); % Append D zeros to the input data
Tf2 = Tf2(D+1:end);  
Tf3 = filter(df,[T(:,4); zeros(D,1)]); % Append D zeros to the input data
Tf3 = Tf3(D+1:end);  

figure(1)
plot(T(:,1),T(:,2),T(:,1),Tf1,'r','linewidth',1.5);
title('Filtered Waveforms');
xlabel('Time (s)')
legend('Original Noisy Signal','Filtered Signal');
grid on
axis tight

figure(2)
plot(T(:,1),T(:,3),T(:,1),Tf2,'r','linewidth',1.5);
title('Filtered Waveforms');
xlabel('Time (s)')
legend('Original Noisy Signal','Filtered Signal');
grid on
axis tight

figure(3)
plot(T(:,1),T(:,4),T(:,1),Tf3,'r','linewidth',1.5);
title('Filtered Waveforms');
xlabel('Time (s)')
legend('Original Noisy Signal','Filtered Signal');
grid on
axis tight

csvwrite('T1.csv',Tf1');
csvwrite('T2.csv',Tf2');
csvwrite('T3.csv',Tf3');

Tfs1=vec2str(scale*Tf1');
Tfs2=vec2str(scale*Tf2');
Tfs3=vec2str(scale*Tf3');