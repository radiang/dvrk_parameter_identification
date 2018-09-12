function [gen] = bijection(map,gen,ident)

m=sort(map.m_ac,'descend');
 
 Parn = gen.sdp_par.';
 Par1 = sym([]); 
% 
 for i=1:length(m)
    Parn(:,m(i))=[];
    Par1(end+1)=gen.sdp_par(m(i));
 end
 
  X_n=map.E'*transpose(Parn);
  %gen.bij_par = map.Kg*X_n;
  
  
  XB1 = map.Kp*X_n;
  gen.bij_beta = double(ident.P*[Par1, transpose(XB1)]');

  
end
