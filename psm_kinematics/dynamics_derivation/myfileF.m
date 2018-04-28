function expr = myfileF(t,in2)
%MYFILEF
%    EXPR = MYFILEF(T,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    28-Apr-2018 19:57:23

%Version: 1.1
Dq1tt = in2(4,:);
Dq2tt = in2(5,:);
Dq3tt = in2(6,:);
q1t = in2(1,:);
q2t = in2(2,:);
q3t = in2(3,:);
t2 = cos(2.908373974667121e-1);
t3 = abs(Dq1tt);
t4 = abs(Dq2tt);
t5 = cos(q2t);
t6 = cos(q1t);
t7 = sin(q2t);
t8 = sin(2.908373974667121e-1);
t9 = sin(q1t);
t10 = t4.^2;
t11 = t3.^2;
t12 = q3t.^2;
t13 = t5.^2;
t14 = q2t.*2.0;
t15 = sin(t14);
t16 = abs(Dq3tt);
t17 = t2.^2;
t18 = 1.0./Dq2tt;
t19 = 1.0./Dq3tt;
t20 = t16.^2;
t21 = q2t-2.908373974667121e-1;
t22 = cos(t21);
t23 = sin(t21);
t24 = conj(Dq2tt);
t25 = conj(Dq1tt);
t26 = t14-5.816747949334241e-1;
t27 = t14-2.908373974667121e-1;
t28 = cos(t14);
t29 = cos(t26);
t30 = cos(t27);
t31 = q3t.*5.0e3;
t34 = t2.*7.5e2;
t32 = t31-t34+7.37e2;
t33 = 1.0./t32;
t35 = Dq3tt.*4.978729388058083e23;
t36 = q3t.*3.319152925372056e24;
t37 = t7.*1.65738858299709e22;
t38 = Dq3tt.*q3t.*3.377699720527872e24;
t39 = t5.*t9.*1.659576462686028e23;
t40 = t5.*t8.*1.686623388395953e22;
t41 = t7.*t9.*4.841369599423283e22;
t42 = t2.*t10.*4.978729388058083e21;
t43 = t8.*t11.*1.452410879826985e21;
t44 = t8.*t10.*1.452410879826985e21;
t45 = t11.*t13.*2.952379771719002e20;
t46 = t11.*t17.*2.533274790395904e21;
t47 = q3t.*t7.*1.124415592263969e23;
t48 = t12.*1.125899906842624e25;
t49 = t6.*t7.*t8.*1.582441973439163e23;
t50 = t2.*t11.*t13.*4.978729388058083e21;
t51 = q3t.*t5.*t9.*1.125899906842624e24;
t52 = q3t.*t2.*t10.*3.377699720527872e22;
t53 = t2.*t5.*t6.*1.582441973439163e23;
t54 = t5.*t7.*t8.*t11.*4.978729388058083e21;
t55 = Dq3tt.*t8.*t10.*t18.*1.688849860263936e22;
t56 = Dq2tt.*t8.*t19.*t20.*1.688849860263936e22;
t57 = q3t.*t2.*t11.*t13.*3.377699720527872e22;
t58 = t2.*t5.*t7.*t11.*1.452410879826985e21;
t59 = q3t.*t5.*t7.*t8.*t11.*3.377699720527872e22;
t96 = t5.*4.834987046735065e21;
t97 = t2.*4.978729388058083e23;
t98 = t8.*1.452410879826985e23;
t99 = t11.*2.741453683171105e21;
t100 = t10.*5.18766938917031e21;
t101 = t5.*t6.*1.555012979232885e23;
t102 = t6.*t7.*4.536333657192268e22;
t103 = t2.*t7.*1.686623388395953e22;
t104 = t10.*t12.*1.125899906842624e23;
t105 = Dq3tt.*t2.*5.066549580791808e23;
t106 = t11.*t15.*7.136178789549919e20;
t107 = q3t.*t2.*3.377699720527872e24;
t108 = q3t.*t10.*3.319152925372056e22;
t109 = q3t.*t11.*t13.*3.319152925372056e22;
t110 = t7.*t8.*t9.*1.688849860263936e23;
t111 = q3t.*t11.*t15.*4.841369599423283e21;
t112 = Dq3tt.*t10.*t18.*4.841369599423283e21;
t113 = Dq2tt.*t19.*t20.*4.841369599423283e21;
t114 = t11.*t12.*t13.*1.125899906842624e23;
t115 = q3t.*t5.*t6.*1.054961315626109e24;
t116 = t8.*t11.*t13.*1.452410879826985e21;
t117 = t11.*t13.*t17.*5.066549580791808e21;
t118 = t2.*t5.*t9.*1.688849860263936e23;
t119 = t2.*t5.*t7.*t8.*t11.*5.066549580791808e21;
t60 = t35+t36+t37+t38+t39+t40+t41+t42+t43+t44+t45+t46+t47+t48+t49+t50+t51+t52+t53+t54+t55+t56+t57+t58+t59-t96-t97-t98-t99-t100-t101-t102-t103-t104-t105-t106-t107-t108-t109-t110-t111-t112-t113-t114-t115-t116-t117-t118-t119+5.18766938917031e23;
t61 = t5.*3.319152925372056e18;
t62 = t7.*9.682739198846566e17;
t63 = q3t.*t5.*2.251799813685248e19;
t120 = t22.*3.377699720527872e18;
t64 = t61+t62+t63-t120;
t65 = 1.0./t64;
t66 = t23.*1.6e-6;
t67 = t5.*1.4e-7;
t68 = t7.*1.57e-6;
t69 = t22.*(-5.4e-7)+t66+t67+t68-1.31e-6;
t70 = t6.*2.251799813685248e20;
t71 = t9.*2.109922631252218e20;
t72 = Dq1tt.*t5.*9.957458776116167e19;
t73 = Dq1tt.*t7.*2.90482175965397e19;
t74 = Dq1tt.*t23.*t24.*3.377699720527872e18;
t75 = Dq2tt.*t23.*t25.*3.377699720527872e18;
t76 = Dq1tt.*t5.*t24.*9.682739198846566e17;
t77 = Dq2tt.*t5.*t25.*9.682739198846566e17;
t78 = conj(Dq3tt);
t79 = Dq1tt.*t5.*t78.*2.251799813685248e19;
t80 = Dq3tt.*t5.*t25.*2.251799813685248e19;
t81 = Dq1tt.*q3t.*t5.*6.755399441055744e20;
t121 = Dq1tt.*t22.*1.013309916158362e20;
t122 = Dq1tt.*t7.*t24.*3.319152925372056e18;
t123 = Dq2tt.*t7.*t25.*3.319152925372056e18;
t124 = Dq1tt.*q3t.*t7.*t24.*2.251799813685248e19;
t125 = Dq2tt.*q3t.*t7.*t25.*2.251799813685248e19;
t82 = t70+t71+t72+t73+t74+t75+t76+t77+t79+t80+t81-t121-t122-t123-t124-t125;
t83 = Dq2tt.*9.957458776116167e19;
t84 = t5.*2.248831184527937e19;
t85 = t8.*3.377699720527872e20;
t86 = t11.*9.682739198846566e17;
t87 = t10.*9.682739198846566e17;
t88 = Dq2tt.*q3t.*6.755399441055744e20;
t89 = t6.*t7.*2.109922631252218e20;
t90 = t11.*t15.*1.659576462686028e18;
t91 = q3t.*t11.*t15.*1.125899906842624e19;
t92 = Dq3tt.*t10.*t18.*2.251799813685248e19;
t93 = Dq2tt.*t19.*t20.*2.251799813685248e19;
t94 = t8.*t11.*t13.*3.377699720527872e18;
t128 = t7.*t9.*2.251799813685248e20;
t129 = t8.*t11.*3.377699720527872e18;
t130 = t8.*t10.*3.377699720527872e18;
t131 = Dq2tt.*t2.*1.013309916158362e20;
t132 = t11.*t13.*9.682739198846566e17;
t133 = t2.*t5.*t7.*t11.*3.377699720527872e18;
t95 = t83+t84+t85+t86+t87+t88+t89+t90+t91+t92+t93+t94-t128-t129-t130-t131-t132-t133-9.682739198846566e19;
t126 = t8.*3.373629e-2;
t127 = t126-9.671069800000001e-3;
expr = [t33.*t60.*1.106319480470574e-23+t33.*t69.*t95.*2.220446049250313e-16+t65.*t82.*(q3t.*3.19253259528e-2+t2.*8.496959377007999e-2-t8.*1.45066047e-3+t12.*2.249086e-1+t15.*1.3630390159704e-3+t28.*8.836568503935193e-2+t29.*3.484252609664336e-2+t30.*8.496959377007999e-2-sin(t26).*2.9698e-4-sin(t27).*1.45066047e-3-q3t.*t2.*3.373629e-2+q3t.*t15.*9.671069800000001e-3+q3t.*t28.*3.19253259528e-2-q3t.*t30.*3.373629e-2+t12.*t28.*1.124543e-1+t12.*t29.*1.124543e-1+1.553985971373953e-1);t33.*t95.*(q3t.*6.38506519056e-2+t2.*1.6993918754016e-1-t8.*2.90132094e-3+t12.*4.498172e-1-q3t.*t2.*6.747258e-2+8.692326257344275e3).*2.220446049250313e-16+t65.*t69.*t82-t33.*t60.*t127.*4.440892098500626e-20;t33.*t60.*1.997589649249676e-20+t65.*t82.*2.49121e-4-t33.*t95.*t127.*2.220446049250313e-16;-Dq1tt;-Dq2tt;-Dq3tt];
