  /*
  V1.2
  */
  t2 = q1-q2;
  t3 = q1+q2-2.908373974667121E-1;
  t4 = q1-q2+2.908373974667121E-1;
  t5 = q1+q2;
  t6 = sin(t4);
  t7 = sin(t5);
  t8 = sin(t2);
  t9 = sin(t3);
  t10 = cos(t2);
  t11 = t10*1.241934354407637E-2;
  t12 = cos(t3);
  t13 = t12*4.448657744200104E-1;
  t14 = t9*1.696027416975238;
  t15 = cos(t4);
  t16 = t6*1.696027416975238;
  t17 = cos(t5);
  t18 = q3*t7*1.783576467713925;
  t19 = q3*t8*1.783576467713925;
  A0[0][0] = t7*(-2.296311070459763)-t8*2.296311070459763+t11+t13+t14-t15*4.448657744200104E-1+t16-t17*1.241934354407637E-2+t18+t19-cos(q1-2.908373974667121E-1)*2.858059933899959+cos(q1+2.908373974667121E-1)*2.858059933899959-cos(q1)*6.853382919529805E-2+sin(q1)*7.137645436605585E-1-q3*t6*1.505992245645058-q3*t9*1.505992245645058;
  A0[1][0] = t7*(-2.296311070459763)+t8*2.296311070459763-t11+t13+t14+t15*4.448657744200104E-1-t16-t17*1.241934354407637E-2+t18-t19+q3*t6*1.505992245645058-q3*t9*1.505992245645058;
  A0[2][0] = t10*(-1.783576467713925)+t12*1.505992245645058+t15*1.505992245645058-t17*1.783576467713925+4.89585313835491E-1;
