  /*
  V1.2
  */
  t2 = q2*2.0;
  t3 = t2-5.816747949334241E-1;
  t4 = t2-2.908373974667121E-1;
  t5 = cos(t2);
  t6 = sin(t2);
  t7 = sin(t3);
  t8 = q3*q3;
  t9 = cos(t3);
  t10 = sin(t4);
  t11 = cos(t4);
  t12 = sin(q2);
  t13 = q2-2.908373974667121E-1;
  t14 = cos(t13);
  t15 = cos(q2);
  t16 = q2-5.816747949334241E-1;
  t17 = sin(t16);
  t18 = cos(2.908373974667121E-1);
  t19 = qd2*qd2;
  t20 = q3*2.005576063912884;
  t21 = qd1*qd1;
  A0[0][0] = t19*(t12*8.545263697829037E-3-t14*1.328944069394546E-1+t15*1.317998318308207E-1-q3*t14*1.466374173549898E-1+q3*t15*1.987270627402102E-1)+qd1*qd3*(t5*(-5.349232129366322E-1)+t6*2.426928837947746E-1+t7*8.942294864141324E-3+t9*3.754882567407658E-1-t11*1.827921992135684E-1-t18*1.827921992135684E-1+t20+q3*t5*1.218614661423789+q3*t9*7.869614024890949E-1-1.594349561958664E-1)+qd2*qd3*(t12*3.974541254804203E-1-sin(t13)*2.932748347099795E-1)-qd1*qd2*(t5*1.556499079348668E-1+t6*8.863958763758443E-1-t7*5.016373791247762E-1-t9*6.087858634025911E-1+t10*1.604769638809897E-1+t11*8.601165048389373E-2+t12*1.215494698652444E-3+t17*1.215494698652444E-3-q3*t5*4.853857675895491E-1-q3*t6*1.069846425873264+q3*t7*7.509765134815315E-1-q3*t9*1.788458972828265E-2-q3*t10*3.655843984271367E-1+t6*t8*1.218614661423789+t7*t8*7.869614024890949E-1);
  A0[1][0] = t21*(t5*7.782495396743341E-2+t6*4.431979381879222E-1-t7*2.508186895623881E-1-t9*3.043929317012956E-1+t10*8.023848194049483E-2+t11*4.300582524194686E-2+t12*6.077473493262222E-4+t17*6.077473493262222E-4-q3*t5*2.426928837947746E-1-q3*t6*5.349232129366322E-1+q3*t7*3.754882567407658E-1-q3*t9*8.942294864141324E-3-q3*t10*1.827921992135684E-1+t6*t8*6.093073307118946E-1+t7*t8*3.934807012445475E-1)-qd2*qd3*(q3*(-4.011152127825768)+t18*3.655843984271367E-1+3.188699123917329E-1);
  A0[2][0] = -t21*(q3*1.002788031956442-t5*2.674616064683161E-1+t6*1.213464418973873E-1+t7*4.471147432070662E-3+t9*1.877441283703829E-1-t11*9.139609960678419E-2-t18*9.139609960678419E-2+q3*t5*6.093073307118946E-1+q3*t9*3.934807012445475E-1-7.971747809793321E-2)+t19*(t18*1.827921992135684E-1-t20+1.594349561958664E-1);