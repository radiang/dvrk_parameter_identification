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
  t12 = sin(q2);
  t13 = q2-2.908373974667121E-1;
  t14 = sin(t13);
  t15 = cos(q2);
  t16 = t15*8.463400573377098E-3;
  t17 = t12*6.269499792655381E-2;
  t18 = q3*t14*3.17367929934558E-2;
  t19 = t14*(-3.93403368571513E-2)+t16+t17+t18-q3*t12*1.968232691483046E-1-3.756529503560488E-2;
  t20 = sin(2.908373974667121E-1);
  t21 = cos(t13);
  t22 = t15*1.968232691483046E-1;
  t23 = t21*(-3.17367929934558E-2)+t22;
  t24 = t20*(-5.459927962389567E-2)+2.159202992293497E-1;
  A0[0][0] = q3*(-9.507251151667528E-2)-t5*1.78401285907968E-1-t6*3.564446557030897E-2+t7*1.527343357089701E-1+t8*2.745853873178645E-1+t9*7.029523685080908E-2+t10*7.029523685080908E-2-t11*2.994856225769315E-2+t12*8.749163062959059E-2-t20*2.418299501435992E-2-sin(q2-5.816747949334241E-1)*8.749163062959059E-2-sin(t4)*2.418299501435992E-2+q3*t5*1.4636570113582E-1+q3*t6*1.612199667623994E-1-q3*t7*2.414382126524953E-1-q3*t9*5.459927962389567E-2-q3*t10*5.459927962389567E-2+q3*t11*5.470033246695026E-2+t5*t8*1.819975987463189E-1+t7*t8*9.258778857154564E-2+2.165875171983218E-1;
  A0[0][1] = t19;
  A0[0][2] = t23;
  A0[1][0] = t19;
  A0[1][1] = q3*(-1.901450230333506E-1)+t8*5.491707746357291E-1+t9*1.405904737016182E-1-t20*4.836599002871983E-2-q3*t9*1.091985592477913E-1+3.289399268885331E-1;
  A0[1][2] = t24;
  A0[2][0] = t23;
  A0[2][1] = t24;
  A0[2][2] = 5.491707746357291E-1;