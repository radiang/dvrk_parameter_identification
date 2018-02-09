clear all

vel=csvread('data_velocity.csv');
hz=250;
Time=linspace(0,length(vel)-1,length(vel))*1/hz;

Fs=250;
%Fnorm = 300/(Fs/2);  % Normalized frequency
Fnorm=0.005;
df = designfilt('lowpassfir','FilterOrder',70,'CutoffFrequency',Fnorm);

%grpdelay(df,2048,Fs) ;  % plot group delay
D = mean(grpdelay(df)) ;% filter delay in samples

Vf1 = filter(df,[vel(:,1); zeros(D,1)]); % Append D zeros to the input data
Vf1 = Vf1(D+1:end);        
Vf2 = filter(df,[vel(:,2); zeros(D,1)]); % Append D zeros to the input data
Vf2 = Vf2(D+1:end);  
Vf3 = filter(df,[vel(:,3); zeros(D,1)]); % Append D zeros to the input data
Vf3 = Vf3(D+1:end);

Vf(:,1)=Vf1;
Vf(:,2)=Vf2;
Vf(:,3)=Vf3;

figure(1)
plot(Time,vel(:,1),Time,Vf1,'r','linewidth',1.5);
title('Filtered Waveforms Velocity Joint 1');
xlabel('Time (s)')
legend('Original Noisy Signal','Filtered Signal');
grid on
axis tight

figure(2)
plot(Time,vel(:,2),Time,Vf2,'r','linewidth',1.5);
title('Filtered Waveforms Velocity Joint 2');
xlabel('Time (s)')
legend('Original Noisy Signal','Filtered Signal');
grid on
axis tight

figure(3)
plot(Time,vel(:,3),Time,Vf3,'r','linewidth',1.5);
title('Filtered Waveforms Velocity Joint 3');
xlabel('Time (s)')
legend('Original Noisy Signal','Filtered Signal');
grid on
axis tight

for i=1:length(vel)-1
    acc(i,1)=(vel(i+1,1)-vel(i,1))*hz;
    acc(i,2)=(vel(i+1,2)-vel(i,2))*hz;
    acc(i,3)=(vel(i+1,3)-vel(i,3))*hz;
end

for i=1:length(vel)-1
    af(i,1)=(Vf(i+1,1)-Vf(i,1))*hz;
    af(i,2)=(Vf(i+1,2)-Vf(i,2))*hz;
    af(i,3)=(Vf(i+1,3)-Vf(i,3))*hz;
end

figure(4)
plot(Time(1:end-1),[acc(:,1) af(:,1)]);
title('Filtered Waveforms Acceleration Joint 1');
xlabel('Time (s)')
legend('Derivative of Noisy Velocity','Derivative of Filtered Velocity');
grid on
axis tight

figure(5)
plot(Time(1:end-1),[acc(:,2) af(:,2)]);
title('Filtered Waveforms Acceleration Joint 2');
xlabel('Time (s)')
legend('Derivative of Noisy Velocity','Derivative of Filtered Velocity');
grid on
axis tight

figure(6)
plot(Time(1:end-1),[acc(:,3) af(:,3)]);
title('Filtered Waveforms Acceleration Joint 3');
xlabel('Time (s)')
legend('Derivative of Noisy Velocity','Derivative of Filtered Velocity');
grid on
axis tight

csvwrite('data_velocity_filtered.csv',Vf);
csvwrite('data_acceleration_filtered.csv',af);