function expr = myfileM(t,in2)
%MYFILEM
%    EXPR = MYFILEM(T,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    12-Apr-2018 19:07:23

%Version: 1.1
q2t = in2(2,:);
q3t = in2(3,:);
t2 = q2t.*2.0;
t3 = t2-2.908373974496499e-1;
t4 = t2-5.816747948992997e-1;
t5 = q2t-5.816747948992997e-1;
t6 = cos(t4);
t7 = q3t.^2;
t8 = sin(t4);
t9 = cos(q2t);
t10 = sin(q2t);
t11 = q2t-2.908373974496499e-1;
t12 = sin(t11);
t13 = t12.*2.000000000043656e-2;
t14 = t9.*1.000000000021828e-2;
t15 = t10.*1.000000000021828e-2;
t16 = t13+t14+t15-2.000000000043656e-2;
expr = reshape([0.0,0.0,0.0,-1.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0,q3t.*1.000000000021828e-2+t6.*6.677000000308908e-3+t7.*5.000000000109139e-3+t8.*4.000000000132786e-4+t9.*1.500000000078217e-3+t10.*1.500000000078217e-3+cos(t2).*1.231315249970066e-2+cos(t3).*1.860749999832478e-2+cos(t5).*1.500000000078217e-3+sin(t2).*4.235850000441133e-3-sin(t3).*4.724999999780266e-3-sin(t5).*1.500000000078217e-3+q3t.*t6.*1.000000000021828e-2+q3t.*t8.*2.000000000043656e-2+t6.*t7.*5.000000000109139e-3+8.272179975756444e-2,t16,5.0e-1,0.0,0.0,0.0,t16,q3t.*2.000000000043656e-2+t7.*5.0e-1+1.553555964201223e-1,2.000000000043656e-2,0.0,0.0,0.0,5.0e-1,2.000000000043656e-2,5.0e-1,0.0,0.0,0.0],[6,6]);
