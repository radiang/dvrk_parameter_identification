%% Recursive Lumping based on Null 
[y, A]=null_find(W);

%while(size(A)>0)
   
 
flag=0;
for i=size(y,3):-1:1
    for j=1:size(y,2)
        if (sum(y(:,j,i)>0)==2)
            x=find(y(:,j,i)>0);
            flag=1;
            break
        end
    end
    if(flag==1)
        break
    end 
    
end

m=[y(x(1),j,i) y(x(2),j,i)];
sort(m)
W(:,m(1))=W(:,m(1))+W(:,m(2));
W(:,m(2))=[]; 

%y=[]; 
%[y, A]=null_find(W);
%end

cond(W)
cond(W,2)