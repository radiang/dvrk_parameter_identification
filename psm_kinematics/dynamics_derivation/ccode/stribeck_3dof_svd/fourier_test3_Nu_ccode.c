  /*
  V1.2
  */
  t2 = q1-q2;
  t3 = q1+q2-2.908373974667121E-1;
  t4 = q1-q2+2.908373974667121E-1;
  t5 = q1+q2;
  t6 = qd2*qd2;
  t7 = sin(t4);
  t8 = sin(t5);
  t9 = sin(t2);
  t10 = sin(t3);
  t11 = cos(q2);
  t12 = sin(q2);
  t13 = q2-2.908373974667121E-1;
  t14 = cos(t13);
  t15 = q2*2.0;
  t16 = t15-5.816747949334241E-1;
  t17 = cos(t16);
  t18 = t15-2.908373974667121E-1;
  t19 = cos(t18);
  t20 = sin(t16);
  t21 = cos(t15);
  t22 = sin(t15);
  t23 = q3*q3;
  t24 = sin(t18);
  t25 = cos(t2);
  t26 = t25*2.361284618621699E-1;
  t27 = cos(t3);
  t28 = t27*5.578968955025231E-1;
  t29 = t10*1.292457872711259;
  t30 = cos(t4);
  t31 = t7*1.292457872711259;
  t32 = cos(t5);
  t33 = qd1*qd1;
  t34 = q3*t8*1.784600497731877;
  t35 = q2-5.816747949334241E-1;
  t36 = cos(t35);
  t37 = q3*t9*1.784600497731877;
  t38 = cos(2.908373974667121E-1);
  A0[0][0] = t8*(-1.941038798896964)-t9*1.941038798896964+t26+t28+t29-t30*5.578968955025231E-1+t31-t32*2.361284618621699E-1+t34+t37-cos(q1-2.908373974667121E-1)*2.677431168419356+cos(q1+2.908373974667121E-1)*2.677431168419356-cos(q1)*7.015342932842435E-2+sin(q1)*2.931717447814206E-1-qd1*qd3*1.579071418675643E-1-q3*t7*1.537044053217864-q3*t10*1.537044053217864+t6*t11*7.944638112413993E-2-t6*t12*1.10762159702361E-2-t6*t14*2.9808282764953E-2-qd2*qd3*sin(t13)*1.478194119858752E-1+q3*qd1*qd3*1.106395730528833+qd1*qd2*t11*1.639243572501646E-1-qd2*qd3*t12*5.151728358249351E-1-qd1*qd2*t17*1.661707860782319E-1-qd1*qd3*t17*3.836690360371641E-1-qd1*qd2*t19*7.843005129906617E-2-qd1*qd2*t20*5.971737017195071E-1-qd1*qd3*t19*1.092612549631762E-1-qd1*qd2*t21*1.131749795650166E-1+qd1*qd3*t20*1.371970936899062E-1+qd1*qd2*t22*6.812594265605478E-1+qd1*qd3*t21*2.257618941695998E-1+qd1*qd3*t22*2.614335043302206E-1-qd1*qd2*t24*2.376782202730977E-1-qd1*qd2*t36*1.639243572501646E-1-qd1*qd3*t38*1.092612549631762E-1-q3*t6*t11*2.575864179124676E-1-q3*t6*t14*7.390970599293761E-2+q3*qd1*qd2*t17*2.743941873798124E-1+q3*qd1*qd3*t17*3.779873641076584E-1+q3*qd1*qd2*t20*7.673380720743282E-1+q3*qd1*qd2*t21*5.228670086604411E-1-q3*qd1*qd2*t22*4.515237883391997E-1+q3*qd1*qd3*t21*7.284083664211743E-1+q3*qd1*qd2*t24*2.185225099263523E-1-qd1*qd2*t20*t23*3.779873641076584E-1-qd1*qd2*t22*t23*7.284083664211743E-1;
  A0[1][0] = t8*(-1.941038798896964)+t9*1.941038798896964-t26+t28+t29+t30*5.578968955025231E-1-t31-t32*2.361284618621699E-1+t34-t37-qd2*qd3*3.158142837351285E-1+q3*t7*1.537044053217864-q3*t10*1.537044053217864-t11*t33*8.196217862508232E-2+t17*t33*8.308539303911593E-2+t19*t33*3.921502564953309E-2+t20*t33*2.985868508597536E-1+t21*t33*5.658748978250832E-2-t22*t33*3.406297132802739E-1+t24*t33*1.188391101365488E-1+t33*t36*8.196217862508232E-2+q3*qd2*qd3*2.212791461057665-qd2*qd3*t38*2.185225099263523E-1-q3*t17*t33*1.371970936899062E-1-q3*t20*t33*3.836690360371641E-1-q3*t21*t33*2.614335043302206E-1+q3*t22*t33*2.257618941695998E-1-q3*t24*t33*1.092612549631762E-1+t20*t23*t33*1.889936820538292E-1+t22*t23*t33*3.642041832105872E-1;
  A0[2][0] = t6*1.579071418675643E-1-t25*1.784600497731877+t27*1.537044053217864+t30*1.537044053217864-t32*1.784600497731877+t33*7.895357093378213E-2-q3*t6*1.106395730528833-q3*t33*5.531978652644164E-1+t6*t38*1.092612549631762E-1+t17*t33*1.91834518018582E-1+t19*t33*5.463062748158808E-2-t20*t33*6.859854684495309E-2-t21*t33*1.128809470847999E-1-t22*t33*1.307167521651103E-1+t33*t38*5.463062748158808E-2-q3*t17*t33*1.889936820538292E-1-q3*t21*t33*3.642041832105872E-1;
