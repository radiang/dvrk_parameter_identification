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
  t26 = t25*8.730835489583073E-1;
  t27 = cos(t3);
  t28 = t27*1.114231277432615;
  t29 = t10*9.405159194723972E-1;
  t30 = cos(t4);
  t31 = t7*9.405159194723972E-1;
  t32 = cos(t5);
  t33 = qd1*qd1;
  t34 = q3*t8*1.758378739211934;
  t35 = q2-5.816747949334241E-1;
  t36 = cos(t35);
  t37 = q3*t9*1.758378739211934;
  t38 = cos(2.908373974667121E-1);
  A0[0][0] = t8*(-1.831651997170329)-t9*1.831651997170329+t26+t28+t29-t30*1.114231277432615+t31-t32*8.730835489583073E-1+t34+t37-cos(q1-2.908373974667121E-1)*2.726821663186681+cos(q1+2.908373974667121E-1)*2.726821663186681+cos(q1)*1.653546285283876E-2+sin(q1)*1.784682133984502-qd1*qd3*2.085155656497254E-1-q3*t7*1.16040741972719-q3*t10*1.16040741972719-t6*t11*4.465527305759931E-2-t6*t12*8.117194432127252E-3+t6*t14*5.13565315125898E-2+qd2*qd3*sin(t13)*2.46814546388379E-1+q3*qd1*qd3*1.00307110595819+qd1*qd2*t11*1.669482650930621E-1-qd2*qd3*t12*3.775439270756861E-1-qd1*qd2*t17*2.684951813657903E-1-qd1*qd3*t17*2.96152783689142E-1-qd1*qd2*t19*1.975299711495781E-2-qd1*qd2*t20*4.490519336875785E-1-qd1*qd3*t19*1.076558411762408E-1-qd1*qd2*t21*1.261981761082416E-2+qd1*qd3*t20*2.740099365931226E-1+qd1*qd2*t22*5.080689596019584E-1+qd1*qd3*t21*8.763721803941656E-2+qd1*qd3*t22*6.584332371652605E-2-qd1*qd2*t24*2.242839180208566E-1-qd1*qd2*t36*1.669482650930621E-1-qd1*qd3*t38*1.076558411762408E-1-q3*t6*t11*1.887719635378431E-1+q3*t6*t14*1.234072731941895E-1+q3*qd1*qd2*t17*5.480198731862452E-1+q3*qd1*qd3*t17*2.853654981165846E-1+q3*qd1*qd2*t20*5.923055673782839E-1+q3*qd1*qd2*t21*1.316866474330521E-1-q3*qd1*qd2*t22*1.752744360788331E-1+q3*qd1*qd3*t21*7.177056078416055E-1+q3*qd1*qd2*t24*2.153116823524817E-1-qd1*qd2*t20*t23*2.853654981165846E-1-qd1*qd2*t22*t23*7.177056078416055E-1;
  A0[1][0] = t8*(-1.831651997170329)+t9*1.831651997170329-t26+t28+t29+t30*1.114231277432615-t31-t32*8.730835489583073E-1+t34-t37-qd2*qd3*4.170311312994508E-1+q3*t7*1.16040741972719-q3*t10*1.16040741972719-t11*t33*8.347413254653106E-2+t17*t33*1.342475906828952E-1+t19*t33*9.876498557478907E-3+t20*t33*2.245259668437893E-1+t21*t33*6.309908805412081E-3-t22*t33*2.540344798009792E-1+t24*t33*1.121419590104283E-1+t33*t36*8.347413254653106E-2+q3*qd2*qd3*2.00614221191638-qd2*qd3*t38*2.153116823524817E-1-q3*t17*t33*2.740099365931226E-1-q3*t20*t33*2.96152783689142E-1-q3*t21*t33*6.584332371652605E-2+q3*t22*t33*8.763721803941656E-2-q3*t24*t33*1.076558411762408E-1+t20*t23*t33*1.426827490582923E-1+t22*t23*t33*3.588528039208028E-1;
  A0[2][0] = t6*2.085155656497254E-1-t25*1.758378739211934+t27*1.16040741972719+t30*1.16040741972719-t32*1.758378739211934+t33*1.042577828248627E-1-q3*t6*1.00307110595819-q3*t33*5.015355529790951E-1+t6*t38*1.076558411762408E-1+t17*t33*1.48076391844571E-1+t19*t33*5.382792058812041E-2-t20*t33*1.370049682965613E-1-t21*t33*4.381860901970828E-2-t22*t33*3.292166185826302E-2+t33*t38*5.382792058812041E-2-q3*t17*t33*1.426827490582923E-1-q3*t21*t33*3.588528039208028E-1;
