  /*
  V1.2
  */
  t2 = q2*2.0;
  t3 = t2-5.816747949334241E-1;
  t4 = t2-2.908373974667121E-1;
  t5 = cos(t2);
  t6 = sin(t2);
  t7 = cos(t3);
  t8 = q3*q3;
  t9 = cos(2.908373974667121E-1);
  t10 = cos(t4);
  t11 = sin(t3);
  t12 = cos(q2);
  t13 = q2-2.908373974667121E-1;
  t14 = sin(t13);
  t15 = sin(q2);
  t16 = t14*2.132047966648602E-1;
  t17 = t12*8.076540215998041E-3;
  t18 = q3*t14*1.457757245529746E-1;
  t19 = t15*(-1.790915086027671E-1)+t16+t17+t18-q3*t15*1.878265166511172E-1+6.598815996580486E-2;
  t20 = sin(2.908373974667121E-1);
  t21 = cos(t13);
  t22 = t12*1.878265166511172E-1;
  t23 = t21*(-1.457757245529746E-1)+t22;
  t24 = t20*(-5.090485994028591E-2)+1.801260819979141E-1;
  A0[0][0] = q3*(-1.000393564900865E-1)+t5*5.365809303957792E-1-t6*3.676670714879839E-3-t7*3.141839480777344E-1+t8*2.847689642316226E-1-t9*2.194277301535982E-2-t10*2.194277301535982E-2+t11*2.891257615923579E-1-t12*4.645340343113588E-2-t20*1.689952691273263E-2-cos(q2-5.816747949334241E-1)*4.645340343113588E-2-sin(t4)*1.689952691273263E-2+q3*t5*1.462851534357322E-1+q3*t6*4.918811809072527E-2-q3*t7*2.463245099258186E-1-q3*t9*5.090485994028591E-2-q3*t10*5.090485994028591E-2+q3*t11*1.309379639071888E-1+t5*t8*1.696828664676197E-1+t7*t8*1.150860977640029E-1+3.774398562257764E-1;
  A0[0][1] = t19;
  A0[0][2] = t23;
  A0[1][0] = t19;
  A0[1][1] = q3*(-2.00078712980173E-1)+t8*5.695379284632453E-1-t9*4.388554603071965E-2-t20*3.379905382546527E-2-q3*t9*1.018097198805718E-1+5.347846374310359E-1;
  A0[1][2] = t24;
  A0[2][0] = t23;
  A0[2][1] = t24;
  A0[2][2] = 5.695379284632453E-1;