function [eff] = compare_effort(gen,eff)

foldername=strcat('data/',gen.filename,'/');
testname = 'effort_data2';
csvname = strcat(foldername,testname,'.csv');
q=csvread(csvname);





end