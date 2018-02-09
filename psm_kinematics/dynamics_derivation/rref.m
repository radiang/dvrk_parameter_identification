[R jb]=rref(W);

njb=linspace(1,46,46);

 [a b] = size(W);

total_indeces = 1:b;
notjb = find(ismember(total_indeces,jb) == 0);

 dep3=[];
 flag=1;
 tol=1;
 for i=1:size(jb,2)
    A=W(:,jb(i));
    for j=1:size(notjb,2)
        B=W(:,notjb(j));
       if rank([A B],tol)==1
           %if ~any(dep1(2,:)==j)
             dep3(1,flag)=jb(i);
             dep3(2,flag)=notjb(j);
             flag=flag+1;
           %end
       end
    end
end