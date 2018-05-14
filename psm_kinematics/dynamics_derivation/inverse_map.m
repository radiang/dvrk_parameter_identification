function [X] = inverse_map(map,par_num,codomain)
beta = par_num;
delta = codomain;

beta1 = beta(1:length(map.m_ac));
beta2 = beta(length(map.m_ac)+1:end);

X = map.E*inv(map.Kg)*[beta2; delta];

beta1_fl = fliplr(beta1.').';

for i=1:length(map.m_ac)
b = beta1_fl(i);
i1 = map.m_ac(i);
if i1 ==1
X=  [b;X(i1:end)];
else
X = [X(1:i1-1);b;X(i1:end)];
end


end


%Db = diag([X(7:31).', X(end-4:end).']);


end