
function [Y2,Par2,condition,W]=svd_reduction(Y,Par,dof_num)

syms q1 q2 q3 q4 q5 q6 qd1 qd2 qd3 qd4 qd5 qd6  qdd1 qdd2 qdd3 qdd4 qdd5 qdd6
 
q   = [q1 q2 q3 q4 q5 q6];
qd  = [qd1 qd2 qd3 qd4 qd5 qd6];
qdd = [qdd1 qdd2 qdd3 qdd4 qdd5 qdd6];


tf=2;
ts=0.2;
g=9.8;
par_num=length(Par);

point_num=5;

%Joint Limits
limit_min(1)=-1.54; %rad
limit_max(1)=1.54;
limit_min(2)=-0.84; %rad
limit_max(2)=0.84;  
limit_min(3)=0;     %m
limit_max(3)=0.24; 
limit_min(4)=-1.54; %rad
limit_max(4)=1.54;
limit_min(5)=-1.54; %rad
limit_max(5)=1.54;
limit_min(6)=-1.54; %rad
limit_max(6)=1.54;

%Maximum Velocity 
velocity_min(1)= -0.7 ;%rad/s
velocity_max(1)= 0.7 ;%rad/s
velocity_min(2)= -0.06;
velocity_max(2)= 0.06; %m/s

%Chosen Trajectories
options=7;
for i=1:6
op(i,:)=linspace(limit_min(i),limit_max(i),options); 
end

for i=1:6
opd(i,:)=linspace(velocity_min(1),velocity_max(1),options);
    if i==3
        opd(i,:)=linspace(velocity_min(2),velocity_max(2),options);
    end
end


for dof=1:dof_num
array = [0];
array_v = [0];
for n = 1:point_num
    
temp = randi(options);
temp2 = randi(options);
   
r(n) = op(dof,temp);
rd(n) = opd(dof,temp2);

array(n+1) = r(n);
array_v(n+1) = rd(n);
end
% save_array(dof,:,j)=array(:);
% save_arrayv(dof,:,j)=array_v(:);

[Qe(dof,:),Qed(dof,:),Qedd(dof,:),T]=Trajectory_f(array,array_v,tf,ts);

end

%Enlarge the matrix with random numbers
W=zeros(dof_num*length(Qe),par_num);
xx=zeros(1,dof_num);
xxd = zeros(1,dof_num);
xxdd = zeros(1,dof_num);

for i=1:length(Qe(1,:))
    for j = 1:dof_num
        xx(1,j)=Qe(j,i);
        xxd(1,j)=Qed(j,i);
        xxdd(1,j)=Qedd(j,i);
    end

W(1+(i-1)*dof_num:dof_num+(i-1)*dof_num,:)=double(subs(Y,[q(1:dof_num), qd(1:dof_num), qdd(1:dof_num)],[xx(1,:),xxd(1,:),xxdd(1,:)]));

end

% [Q,R,P] = qr(W);
% 
 c = size(W,2);
 b = rank(W);


[U,S,V]=svd(W);

V1 = V(1:c,1:b);
V2 = V(1:c,b+1:end);

check(1:(c-b),1:(c-b))=V2(end-(c-b-1):end,:);

m = [];
V2t = V2';
tolerance = 10^-7;

for i = 1:size(V2t,2)
 if(abs(sum(V2t(:,i)))<tolerance)
     m(end+1)=i;   
 end
end

m=sort(m,'descend');
 Wn=W;
 
 Yn = Y;
 Parn = Par;

 W1 = [];
 Y1 = sym([]);
 Par1 = sym([]); 
% 
 for i=1:length(m)
    Wn(:,m(i))=[];
    Yn(:,m(i))=[];
    Parn(:,m(i))=[];
    W1(:,end+1)=W(:,m(i));
    Y1(:,end+1)=Y(:,m(i));
    Par1(end+1)=Par(m(i));
 end

[Un,Sn,Vn]=svd(Wn);
 cn = size(Wn,2);
 bn = rank(Wn);
 
V1n = Vn(1:cn,1:bn);
V2n = Vn(1:cn,bn+1:end);

mn = [];
V2tn = V2n';
flag =1;
count = 1;
while flag==1
E = eye(size(V2n,1));
E = E(randperm(size(V2n,1)),:);

V2tot=(E'*V2n);

V21 = V2tot(1:bn,1:cn-bn);
V22 = V2tot(bn+1:end,1:cn-bn);

 temp=rank(V22)
 
 if(temp==(cn-bn)||count==100000)
        flag=0;
 end  
 count=count+1;   
end

 X_n=E'*transpose(Parn);
 X1 =X_n(1:bn);
 X2 = X_n(bn+1:end);

XB1 = X1-V21*inv(V22)*X2;

WB = Wn*E;
YB = Yn*E;

YB1 = YB(:,1:bn);
WB1 = WB(:,1:bn);

Y2 = [Y1, YB1];
W2 = [W1, WB1];
Par2 = [Par1, transpose(XB1)];

condition=cond(W2)

end