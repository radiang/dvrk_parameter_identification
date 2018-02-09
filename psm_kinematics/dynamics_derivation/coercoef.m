F=corrcoef(W)
[x,y]=find(F==1);
 dep3(1,:)=x;
  dep3(2,:)=y;
  
  for i=length(dep3):-1:1
      if dep3(2,i)==dep3(1,i)
          dep3(:,i)=[];
     
          
      end
  end
  
  
  
          