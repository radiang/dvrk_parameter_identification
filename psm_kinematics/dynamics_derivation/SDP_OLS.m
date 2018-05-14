function [gen] = SDP_OLS(gen,ident,dyn,mapz)
   syms u 
   
  nb = length(gen.Par2);
  beta = sym('beta_%d', [nb 1],'real');

  n = nb;
  %% Make Ub
  w = ident.tau(1:size(ident.W)/3,:);
  w = reshape(w.',1,[]);
  
  [Q, R] = qr(ident.W);
  sizew = size(ident.W,2);
  R1 = R(1:sizew,1:sizew);
  
  Q1 = Q(:,1:sizew);
  Q2 = Q(:,sizew+1:end);
  
  p1 = Q1.'*w.';
  p2 = Q2.'*w.';
   
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
  %% Solve SDP
  
cvx_begin sdp
variable u
variable beto(n)
%variable gamma
minimize(u)
%F >= 0
[u - norm(p2)^2 (p1-R1*beto)'; (p1-R1*beto) eye(n)]>=0
%[u - norm(p2)^2, (p1-R1*beto)']>=0
inverse_map(mapz,beto) >=0

cvx_end
  
gen.par_num_sdp = beto;
x = 0 ;

end
