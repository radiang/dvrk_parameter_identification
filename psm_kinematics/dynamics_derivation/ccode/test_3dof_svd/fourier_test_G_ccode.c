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
  t11 = t8*4.763095627794653E-2;
  t12 = cos(t3);
  t13 = cos(t4);
  t14 = t13*3.287330456856145E-1;
  t15 = cos(t5);
  t16 = t15*2.02454706591134E-1;
  t17 = t7*4.763095627794653E-2;
  t18 = q3*t6*1.515613274071361E-1;
  t19 = q3*t7*3.435287074966372E-1;
  t20 = q3*t8*3.435287074966372E-1;
  t21 = q3*t9*1.515613274071361E-1;
  A0[0][0] = q1*6.795089986834384E-1-t6*5.832887655183789E-1-t9*5.832887655183789E-1-t10*2.02454706591134E-1+t11-t12*3.287330456856145E-1+t14+t16+t17+t18+t19+t20+t21-sin(q1-2.908373974667121E-1)*1.84655595628079E-1-sin(q1+2.908373974667121E-1)*1.84655595628079E-1-cos(q1)*4.880959333766999E-2+sin(q1)*4.422014521957717E-1;
  A0[1][0] = q2*1.152674076488623+t6*5.832887655183789E-1-t9*5.832887655183789E-1+t10*2.02454706591134E-1-t11-t12*3.287330456856145E-1-t14+t16+t17-t18+t19-t20+t21;
  A0[2][0] = t10*(-3.435287074966372E-1)-t12*1.515613274071361E-1-t13*1.515613274071361E-1-t15*3.435287074966372E-1;