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
  t11 = sin(q2);
  t12 = q2-2.908373974667121E-1;
  t13 = cos(t12);
  t14 = q2*2.0;
  t15 = t14-5.816747949334241E-1;
  t16 = cos(t15);
  t17 = t14-2.908373974667121E-1;
  t18 = cos(t17);
  t19 = sin(t15);
  t20 = cos(q2);
  t21 = cos(t14);
  t22 = sin(t14);
  t23 = q3*q3;
  t24 = sin(t17);
  t25 = cos(t2);
  t26 = cos(t3);
  t27 = t10*3.038338380971479;
  t28 = cos(t4);
  t29 = t28*7.012781887087771E-1;
  t30 = t7*3.038338380971479;
  t31 = cos(t5);
  t32 = t31*1.472718412042649;
  t33 = qd1*qd1;
  t34 = q3*t8*2.474426749202548;
  t35 = q3*t9*2.474426749202548;
  t36 = q2-5.816747949334241E-1;
  t37 = sin(t36);
  t38 = cos(2.908373974667121E-1);
  A0[0][0] = t8*(-2.721227048906702)-t9*2.721227048906702-t25*1.472718412042649-t26*7.012781887087771E-1+t27+t29+t30+t32+t34+t35-sin(q1-2.908373974667121E-1)*3.391158210577204E-1-sin(q1+2.908373974667121E-1)*3.391158210577204E-1-cos(q1)*1.598521877010351E-2+sin(q1)*6.441903349878679E-1-qd1*qd3*2.666389062917804E-1-q3*t7*2.376385154168555-q3*t10*2.376385154168555-t6*t11*2.743852200846444E-3+t6*t13*1.29905543061062E-1-t6*t20*1.376879897186373E-1+qd2*qd3*sin(t12)*3.137128209792096E-1+q3*qd1*qd3*1.59436685901596-qd1*qd2*t11*2.076219312598288E-2-qd2*qd3*t11*1.27621032597509E-1+qd1*qd2*t16*1.12649261594183-qd1*qd3*t16*8.384592620385699E-1+qd1*qd2*t18*1.803328667807326E-1+qd1*qd2*t19*1.17941198760306-qd1*qd3*t18*1.514955152572988E-1-qd1*qd3*t19*1.724571872232809E-1-qd1*qd2*t21*3.909008012349778E-1-qd1*qd2*t22*2.029820926962398+qd1*qd3*t21*5.718203557467895E-1+qd1*qd3*t22*8.071867008874725E-1+qd1*qd2*t24*1.715461067240368E-1-qd1*qd2*t37*2.076219312598288E-2-qd1*qd3*t38*1.514955152572988E-1+q3*t6*t13*1.568564104896048E-1-q3*t6*t20*6.38105162987545E-2-q3*qd1*qd2*t16*3.449143744465618E-1+q3*qd1*qd3*t16*5.843967573006344E-1+q3*qd1*qd2*t19*1.67691852407714+q3*qd1*qd2*t21*1.614373401774945-q3*qd1*qd2*t22*1.143640711493579+q3*qd1*qd3*t21*1.009970101715326+q3*qd1*qd2*t24*3.029910305145977E-1-qd1*qd2*t19*t23*5.843967573006344E-1-qd1*qd2*t22*t23*1.009970101715326;
  A0[1][0] = t8*(-2.721227048906702)+t9*2.721227048906702+t25*1.472718412042649-t26*7.012781887087771E-1+t27-t29-t30+t32+t34-t35-qd2*qd3*5.332778125835608E-1+q3*t7*2.376385154168555-q3*t10*2.376385154168555+t11*t33*1.038109656299144E-2-t16*t33*5.63246307970915E-1-t18*t33*9.016643339036629E-2-t19*t33*5.8970599380153E-1+t21*t33*1.954504006174889E-1+t22*t33*1.014910463481199-t24*t33*8.577305336201842E-2+t33*t37*1.038109656299144E-2+q3*qd2*qd3*3.18873371803192-qd2*qd3*t38*3.029910305145977E-1+q3*t16*t33*1.724571872232809E-1-q3*t19*t33*8.384592620385699E-1-q3*t21*t33*8.071867008874725E-1+q3*t22*t33*5.718203557467895E-1-q3*t24*t33*1.514955152572988E-1+t19*t23*t33*2.921983786503172E-1+t22*t23*t33*5.049850508576628E-1;
  A0[2][0] = t6*2.666389062917804E-1-t25*2.474426749202548+t26*2.376385154168555+t28*2.376385154168555-t31*2.474426749202548+t33*1.333194531458902E-1-q3*t6*1.59436685901596-q3*t33*7.971834295079799E-1+t6*t38*1.514955152572988E-1+t16*t33*4.192296310192849E-1+t18*t33*7.574775762864941E-2+t19*t33*8.622859361164045E-2-t21*t33*2.859101778733947E-1-t22*t33*4.035933504437363E-1+t33*t38*7.574775762864941E-2-q3*t16*t33*2.921983786503172E-1-q3*t21*t33*5.049850508576628E-1;