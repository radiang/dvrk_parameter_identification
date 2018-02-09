A=null(W);
flag=1;
x=[];
y=[];
for j=7:-1:1

for i=1:size(A,2)
    x=find(abs(A(:,i))>j*0.1);
    y(1:size(x),i,j)=x; 
    x=[];
end

end
