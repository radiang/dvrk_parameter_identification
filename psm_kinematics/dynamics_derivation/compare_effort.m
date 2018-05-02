function [eff] = compare_effort(gen,eff)

foldername=strcat('data/',gen.filename,'/');
testname = strcat(gen.csvfilename,'_effort_results.csv');
csvname = strcat(foldername,testname);
q=csvread(csvname);

T = linspace(1,length(q),length(q))./max(eff.T);
%error
eff.error(:,:) = eff.joint(:,1:length(q))-q(:,1:3).';

% Plot 
% figure()
% 
% subplot(3,1,1)
% plot(eff.T,eff.joint(1,:),eff.T(1:length(q)),q(:,1));
% title('Desired vs Actual joint positions of PSM with open loop efforts')
% legend('desired','actual')
% subplot(3,1,2)
% plot(eff.T,eff.joint(2,:),eff.T(1:length(q)),q(:,2));
% legend('desired','actual')
% subplot(3,1,3)
% plot(eff.T,eff.joint(3,:),eff.T(1:length(q)),q(:,3));
% legend('desired','actual')
% 
% 
% xlabel('Time [s]')
% ylabel('Joint Angle [rad]')
% 
% %Plot
% figure()
% 
% subplot(3,1,1)
% plot(eff.T(1:length(q)),eff.error(1,:));
% title('Error of joint positions of PSM with open loop efforts')
% subplot(3,1,2)
% plot(eff.T(1:length(q)),eff.error(2,:));
% subplot(3,1,3)
% plot(eff.T(1:length(q)),eff.error(3,:));
% 




end