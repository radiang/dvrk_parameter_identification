function Db = Db_function(I,l,m)
% 
% % %with I in link frame Lk
%  S = [0, -l(3), l(2); l(3), 0 -l(1);-l(2), l(1), 0];
%  Db = [I, S;S, eye(3)*m];

%with I in Inertia frame
% This is no reference at all
Db = [I, zeros(3); zeros(3), diag([m,m,m])];



end