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
  t9 = q2-5.816747949334241E-1;
  t10 = cos(2.908373974667121E-1);
  t11 = cos(t3);
  t12 = sin(t6);
  t13 = sin(t3);
  t14 = cos(q2);
  t15 = sin(q2);
  t16 = q2-2.908373974667121E-1;
  t17 = sin(t16);
  t18 = sin(t9);
  t19 = cos(t16);
  t20 = t15*5.654631427709586E-2;
  t21 = q3*t17*7.517197431331207E-2;
  t22 = qd1*t11*1.104353706054793E-1;
  t23 = q3*qd1*t13*4.206473969346578E-2;
  t24 = q3*qd1*t4*1.47470064140218E-1;
  t25 = q3*qd1*t5*9.246377727269117E-1;
  t26 = qd1*t8*t12*6.1861766288627E-2;
  t27 = sin(2.908373974667121E-1);
  t28 = t14*1.962732185945708E-1;
  t29 = qd1*t5*7.373503207010898E-2;
  t30 = q3*qd1*7.835403268959226E-2;
  t31 = qd1*t7*2.170446773036259E-1;
  t32 = q3*qd1*t4*1.402157989782193E-1;
  t33 = t27*1.051618492336644E-2;
  t34 = q3*qd2*1.567080653791845E-1;
  A0[0][0] = q3*1.22637104529915E-1-qd3*2.4527420905983E-1-t4*3.04969818341196E-2+t5*1.969286973527256E-2-t7*7.887138692524833E-4-t8*1.958850817239807E-2-t10*3.467391647725919E-2-t11*3.467391647725919E-2-t13*2.760884265136983E-2-t14*5.652722315145276E-3-t27*2.760884265136983E-2-cos(t9)*5.652722315145276E-3+q3*qd3*7.835403268959226E-2+q3*t4*2.311594431817279E-1-q3*t5*3.686751603505449E-2-q3*t7*1.085223386518129E-1+q3*t10*1.051618492336644E-2+q3*t11*1.051618492336644E-2+q3*t12*6.708837667053356E-2-qd2*t4*7.877147894109025E-2-qd2*t5*1.219879273364784E-1-qd3*t4*4.623188863634558E-1+qd3*t5*7.373503207010898E-2+qd3*t7*2.170446773036259E-1+qd2*t11*1.104353706054793E-1-qd3*t10*2.103236984673289E-2-qd2*t12*3.154855477009933E-3-qd3*t11*2.103236984673289E-2-qd2*t13*1.386956659090368E-1-qd3*t12*1.341767533410671E-1-qd2*t15*1.130544463029055E-2-qd2*t18*1.130544463029055E-2-t4*t8*3.505394974455481E-2+t7*t8*1.546544157215675E-2+q3*qd2*t4*1.47470064140218E-1+q3*qd2*t5*9.246377727269117E-1+q3*qd3*t4*1.402157989782193E-1-q3*qd2*t7*2.683535066821343E-1-q3*qd3*t7*6.1861766288627E-2-q3*qd2*t12*4.340893546072517E-1+q3*qd2*t13*4.206473969346578E-2-qd2*t5*t8*1.402157989782193E-1+qd2*t8*t12*6.1861766288627E-2-1.53729562059832E-1;
  A0[0][1] = t14*(-3.973047675957156E-2)-t17*3.428948186011343E-2+t20+t21+t22+t23+t24+t25+t26-q3*t15*1.962732185945708E-1-qd1*t4*7.877147894109025E-2-qd1*t5*1.219879273364784E-1-qd1*t12*3.154855477009933E-3-qd1*t13*1.386956659090368E-1-qd1*t15*1.130544463029055E-2-qd2*t14*2.261852571083834E-1-qd2*t15*1.589219070382863E-1+qd3*t15*7.850928743782833E-1-qd1*t18*1.130544463029055E-2-qd3*t17*3.006878972532483E-1+qd2*t19*1.371579274404537E-1-q3*qd1*t7*2.683535066821343E-1-q3*qd1*t12*4.340893546072517E-1+q3*qd2*t14*7.850928743782833E-1-q3*qd2*t19*3.006878972532483E-1-qd1*t5*t8*1.402157989782193E-1+2.768901199983599E-2;
  A0[0][2] = qd1*(-2.4527420905983E-1)-t19*7.517197431331207E-2+t28+t29+t30+t31+t32-qd1*t4*4.623188863634558E-1-qd1*t10*2.103236984673289E-2-qd1*t11*2.103236984673289E-2-qd1*t12*1.341767533410671E-1+qd2*t15*7.850928743782833E-1-qd2*t17*3.006878972532483E-1-q3*qd1*t7*6.1861766288627E-2;
  A0[1][0] = t14*(-3.973047675957156E-2)-t17*3.428948186011343E-2+t20+t21-t22-t23-t24-t25-t26-q3*t15*1.962732185945708E-1+qd1*t4*7.877147894109025E-2+qd1*t5*1.219879273364784E-1+qd1*t12*3.154855477009933E-3+qd1*t13*1.386956659090368E-1+qd1*t15*1.130544463029055E-2+qd1*t18*1.130544463029055E-2+q3*qd1*t7*2.683535066821343E-1+q3*qd1*t12*4.340893546072517E-1+qd1*t5*t8*1.402157989782193E-1+2.768901199983599E-2;
  A0[1][1] = q3*2.4527420905983E-1-qd3*4.9054841811966E-1-t8*3.917701634479613E-2-t10*6.934783295451838E-2-t27*5.521768530273965E-2+q3*qd3*1.567080653791845E-1+q3*t10*2.103236984673289E-2-qd3*t10*4.206473969346578E-2-2.345828702378134E-1;
  A0[1][2] = qd2*(-4.9054841811966E-1)+t33+t34-qd2*t10*4.206473969346578E-2+3.022086063547907E-2;
  A0[2][0] = qd1*2.4527420905983E-1-t19*7.517197431331207E-2+t28-t29-t30-t31-t32+qd1*t4*4.623188863634558E-1+qd1*t10*2.103236984673289E-2+qd1*t11*2.103236984673289E-2+qd1*t12*1.341767533410671E-1+q3*qd1*t7*6.1861766288627E-2;
  A0[2][1] = qd2*4.9054841811966E-1+t33-t34+qd2*t10*4.206473969346578E-2+3.022086063547907E-2;
  A0[2][2] = 2.023731847318903;