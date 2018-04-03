function [y, A]=null_find(W)
A=null(W);
flag=1;
x=[];
y=[];
for j=200:-1:1

for i=1:size(A,2)
    x=find(abs(A(:,i))>j*0.005);
    y(1:size(x),i,j)=x; 
    h=x;
    x=[];
end

end

end 
