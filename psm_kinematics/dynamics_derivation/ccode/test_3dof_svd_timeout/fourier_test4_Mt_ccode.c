  /*
  V1.2
  */
  t2 = q2*2.0;
  t3 = t2-2.908373974667121E-1;
  t4 = cos(t2);
  t5 = sin(t2);
  t6 = t2-5.816747949334241E-1;
  t7 = cos(t6);
  t8 = q3*q3;
  t9 = cos(2.908373974667121E-1);
  t10 = cos(t3);
  t11 = cos(q2);
  t12 = q2-2.908373974667121E-1;
  t13 = sin(t12);
  t14 = sin(q2);
  t15 = t13*2.457822855864688E-2;
  t16 = q3*t14*2.451759038923274E-2;
  t17 = t11*(-2.929146842514746E-2)-t14*7.063517791137953E-3+t15+t16-q3*t13*7.167092090518014E-2+1.828640560413974E-2;
  t18 = sin(2.908373974667121E-1);
  t19 = cos(t12);
  t20 = t19*7.167092090518014E-2;
  t21 = t11*(-2.451759038923274E-2)+t20;
  t22 = t18*(-3.509411368238982E-2)+2.260411337389089E-1;
  A0[0][0] = q3*(-1.507913519739999E-1)+t4*1.44240132283077E-1-t5*4.779703710139219E-2+t7*2.632058526179236E-3+t8*2.225864565939968E-1-t9*5.519840188030211E-2-t10*5.519840188030211E-2-t11*9.046760060920608E-3+t18*9.95534073709602E-2-cos(q2-5.816747949334241E-1)*9.046760060920608E-3+sin(t3)*9.95534073709602E-2+q3*t4*3.679893458686807E-1+q3*t5*2.308884242645694E-1-q3*t7*5.187806978426807E-1-q3*t9*3.509411368238982E-2-q3*t10*3.509411368238982E-2+t4*t8*1.169803789412994E-1+t7*t8*1.056060776526974E-1-q3*sin(t6)*4.847290525660526E-3+3.550221639472068E-1;
  A0[0][1] = t17;
  A0[0][2] = t21;
  A0[1][0] = t17;
  A0[1][1] = q3*(-3.015827039479999E-1)+t8*4.451729131879936E-1-t9*1.103968037606042E-1+t18*1.991068147419204E-1-q3*t9*7.018822736477964E-2+5.149110705373579E-1;
  A0[1][2] = t22;
  A0[2][0] = t21;
  A0[2][1] = t22;
  A0[2][2] = 4.451729131879936E-1;