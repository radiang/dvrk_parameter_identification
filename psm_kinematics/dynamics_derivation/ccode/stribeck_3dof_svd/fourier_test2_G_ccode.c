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
  t11 = t10*9.209676191740891E-1;
  t12 = cos(t3);
  t13 = t12*6.166760136249095E-1;
  t14 = cos(t4);
  t15 = cos(t5);
  t16 = q3*t7*1.617926283040782;
  t17 = q3*t8*1.617926283040782;
  A0[0][0] = t6*(-8.111837967757867E-1)-t7*5.897037263203755E-2-t8*5.897037263203755E-2-t9*8.111837967757867E-1+t11+t13-t14*6.166760136249095E-1-t15*9.209676191740891E-1+t16+t17+cos(q1-2.908373974667121E-1)*2.956076095947223-cos(q1+2.908373974667121E-1)*2.956076095947223-cos(q1)*1.407696426408592E-2+sin(q1)*3.406573346317634-q3*t6*1.614066522303633-q3*t9*1.614066522303633;
  A0[1][0] = t6*8.111837967757867E-1-t7*5.897037263203755E-2+t8*5.897037263203755E-2-t9*8.111837967757867E-1-t11+t13+t14*6.166760136249095E-1-t15*9.209676191740891E-1+t16-t17+q3*t6*1.614066522303633-q3*t9*1.614066522303633;
  A0[2][0] = t10*(-1.617926283040782)+t12*1.614066522303633+t14*1.614066522303633-t15*1.617926283040782;
