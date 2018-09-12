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
  t26 = t9*1.071859062803326;
  t27 = cos(t3);
  t28 = cos(t4);
  t29 = t28*5.809156491214799E-1;
  t30 = cos(t5);
  t31 = t30*2.196766745951028E-1;
  t32 = t8*1.071859062803326;
  t33 = qd1*qd1;
  t34 = q3*t8*1.833125950894082;
  t35 = q3*t9*1.833125950894082;
  t36 = q2-5.816747949334241E-1;
  t37 = sin(t36);
  t38 = cos(2.908373974667121E-1);
  A0[0][0] = t7*(-8.598564333035772E-1)-t10*8.598564333035772E-1-t25*2.196766745951028E-1+t26-t27*5.809156491214799E-1+t29+t31+t32+t34+t35+sin(q1-2.908373974667121E-1)*6.477716830653838E-1+sin(q1+2.908373974667121E-1)*6.477716830653838E-1-cos(q1)*8.987324977736341E-3-sin(q1)*1.314944299479566-qd1*qd3*2.106932395458963E-1-q3*t7*1.808206876323754-q3*t10*1.808206876323754+t6*t11*7.306020157229516E-3-t6*t13*2.052275027097451E-2+t6*t20*2.851519697487739E-2-qd2*qd3*sin(t12)*5.364971350826875E-1+q3*qd1*qd3*1.192885956685365+qd1*qd2*t11*3.965949079992146E-2+qd2*qd3*t11*3.39814891033931E-1+qd1*qd2*t16*5.905018594585585E-1+qd1*qd3*t16*1.438345890830011E-1+qd1*qd2*t18*2.68991846442983E-2+qd1*qd2*t19*5.562384449852259E-1-qd1*qd3*t18*1.122322010751479E-1-qd1*qd3*t19*1.428578279982415E-1-qd1*qd2*t21*1.958264628851884E-1-qd1*qd2*t22*1.027654560241397-qd1*qd3*t21*3.545278286288974E-1+qd1*qd3*t22*3.191171213308341E-1-qd1*qd2*t24*1.063583485886692E-1+qd1*qd2*t37*3.965949079992146E-2-qd1*qd3*t38*1.122322010751479E-1-q3*t6*t13*2.682485675413437E-1+q3*t6*t20*1.699074455169655E-1-q3*qd1*qd2*t16*2.85715655996483E-1+q3*qd1*qd3*t16*4.446712828510457E-1-q3*qd1*qd2*t19*2.876691781660021E-1+q3*qd1*qd2*t21*6.382342426616683E-1+q3*qd1*qd2*t22*7.090556572577948E-1+q3*qd1*qd3*t21*7.482146738343192E-1+q3*qd1*qd2*t24*2.244644021502957E-1-qd1*qd2*t19*t23*4.446712828510457E-1-qd1*qd2*t22*t23*7.482146738343192E-1;
  A0[1][0] = t7*8.598564333035772E-1-t10*8.598564333035772E-1+t25*2.196766745951028E-1-t26-t27*5.809156491214799E-1-t29+t31+t32+t34-t35-qd2*qd3*4.213864790917926E-1+q3*t7*1.808206876323754-q3*t10*1.808206876323754-t11*t33*1.982974539996073E-2-t16*t33*2.952509297292792E-1-t18*t33*1.344959232214915E-2-t19*t33*2.78119222492613E-1+t21*t33*9.79132314425942E-2+t22*t33*5.138272801206986E-1+t24*t33*5.317917429433461E-2-t33*t37*1.982974539996073E-2+q3*qd2*qd3*2.38577191337073-qd2*qd3*t38*2.244644021502957E-1+q3*t16*t33*1.428578279982415E-1+q3*t19*t33*1.438345890830011E-1-q3*t21*t33*3.191171213308341E-1-q3*t22*t33*3.545278286288974E-1-q3*t24*t33*1.122322010751479E-1+t19*t23*t33*2.223356414255229E-1+t22*t23*t33*3.741073369171596E-1;
  A0[2][0] = t6*2.106932395458963E-1-t25*1.833125950894082+t27*1.808206876323754+t28*1.808206876323754-t30*1.833125950894082+t33*1.053466197729482E-1-q3*t6*1.192885956685365-q3*t33*5.964429783426824E-1+t6*t38*1.122322010751479E-1-t16*t33*7.191729454150053E-2+t18*t33*5.611610053757394E-2+t19*t33*7.142891399912074E-2+t21*t33*1.772639143144487E-1-t22*t33*1.595585606654171E-1+t33*t38*5.611610053757394E-2-q3*t16*t33*2.223356414255229E-1-q3*t21*t33*3.741073369171596E-1;