function [gen] = SDP_OLS(gen,ident,dyn,mapz)
   syms u 
   
  nb = length(gen.Par2);
  beta = sym('beta_%d', [nb 1],'real');

  n = nb;
  nd = mapz.cn-mapz.bn;
  %% fdsa

%r_var2 = ident.scale;
huge = [];

for i = 1:length(ident.W)/gen.dof
   huge = [huge,ident.r_var2];
end

G = diag(huge);
  %% Scale Data
  w = ident.tauf(1:size(ident.W)/3,:);
  w = reshape(w.',1,[]).';
  
  W = ident.W;
  
  %% Make Ub
  [Q, R] = qr(W);
  sizew = size(W,2);
  R1 = R(1:sizew,1:sizew);
  
  Q1 = Q(:,1:sizew);
  Q2 = Q(:,sizew+1:end);
  
  p1 = Q1.'*w;
  p2 = Q2.'*w;
   
  Up_11 = u - norm(p2)^2;
  Up_12 = (p1-R1*beta);
  Up_22 = eye;
  
  Up = [Up_11, Up_12.';
        Up_12, eye(nb)];
    
    
  %% Make Db
  % links 1 2 3 4 6 11 (6 total) Db size of 6*N x 6*N = 36 x 36;
  map = [1 2 3 4 6 11];
  array = [];
  for i= 1:length(map)
      
  Dk(:,:,i) = Db_function(dyn.I(:,:,map(i)), dyn.l_cg(map(i),:),dyn.M(map(i)));
  
  if i == 1
  Db = Dk(:,:,i);
  
  else
      Db = blkdiag(Db,Dk(:,:,i));
  end
  end
  
F = blkdiag(Up,Db);

nx = 0;

%% Find Inertia Strings in gen.Par

index = ["xx", "yy","zz","xy","xz","yz"];
for j = 1:length(index)
for i = 1:length(dyn.M)
    temp = sprintf('dyn.I%s(i)',index(j));
    temp2 = sprintf('arr(%d,%d)',[j,i]);
    %arr(j,i)
    eval(strcat( 'x = find(gen.Par ==', temp,');'));
   
   if isempty(x)
      arr(j,i) = 0;
      
   else
       arr(j,i) = x;
   end
  
end
end


for i = 1:length(dyn.M)
      x = find(gen.Par == dyn.M(i));
   if isempty(x)
      mass(i) = 0;
   else
       mass(i) = x;
   end
  
end
  %% Solve SDP

  counter = 1;
  
cvx_begin sdp
variable u
variable beto(n)
variable delto(nd)
minimize(u)
%F >= 0
[u - norm(p2)^2 (p1-R1*beto)'; (p1-R1*beto) eye(n)]>=0
%[u - norm(p2)^2, (p1-R1*beto)']>=0

X=inverse_map(mapz,beto,delto)
% Mass Constraints
for i = 1:length(mass)
    if sum(mass(i)) ~=0
        if counter ==1
            temp = X(mass(i));
        else
            temp(end+1) = X(mass(i)); 
        end
      counter = counter +1;
    end 
end
diag(temp)>=0

% Inertia Constraints
for i = 1:length(arr)
    if sum(arr(:,i)) ~=0
     
       for j = 1:length(index)
          if arr(j,i) ~=0
              if j<=3
              D(j,j,i)= X(arr(j,i));
              elseif j==4
              D(1,2,i)= X(arr(j,i));
              D(2,1,i)= X(arr(j,i));
              elseif j ==5 
              D(1,3,i)= X(arr(j,i));
              D(3,1,i)= X(arr(j,i));
              else
              D(2,3,i)= X(arr(j,i));
              D(3,2,i)= X(arr(j,i));
              end
          end
       end
        D(:,:,i) >=0 
    end
      
end

cvx_end
  
gen.sdp_par2 = ident.P*beto;
gen.sdp_delta = delto;
gen.sdp_u = u;
x = 0 ;

gen.sdp_par = inverse_map(mapz,beto,delto);
end
