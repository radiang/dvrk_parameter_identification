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
  t26 = t25*6.415601204027139E-1;
  t27 = cos(t3);
  t28 = t27*6.483473461732902E-1;
  t29 = t10*1.884422432963664E-1;
  t30 = cos(t4);
  t31 = t7*1.884422432963664E-1;
  t32 = cos(t5);
  t33 = qd1*qd1;
  t34 = q3*t8*1.9215503790456;
  t35 = q2-5.816747949334241E-1;
  t36 = cos(t35);
  t37 = q3*t9*1.9215503790456;
  t38 = cos(2.908373974667121E-1);
  A0[0][0] = t8*(-9.948823513618807E-1)-t9*9.948823513618807E-1+t26+t28+t29-t30*6.483473461732902E-1+t31-t32*6.415601204027139E-1+t34+t37-cos(q1-2.908373974667121E-1)*3.086500892682854+cos(q1+2.908373974667121E-1)*3.086500892682854+cos(q1)*1.791864367141647E-2+sin(q1)*2.020191030505377-qd1*qd3*2.195376464075602E-1-q3*t7*1.161870910921767-q3*t10*1.161870910921767-t6*t11*7.129038783843219E-2-t6*t12*1.095219234168723E-2+t6*t14*8.245919676755961E-2+qd2*qd3*sin(t13)*3.357242645116726E-1+q3*qd1*qd3*1.070031674643251+qd1*qd2*t11*1.889694424091543E-1-qd2*qd3*t12*5.094042949621966E-1-qd1*qd2*t17*2.201042033447503E-1-qd1*qd3*t17*1.172230886091048E-1-qd1*qd2*t19*1.593530008221357E-2-qd1*qd2*t20*4.343897440201295E-1-qd1*qd3*t19*1.176459415742204E-1-qd1*qd2*t21*1.997300875518649E-2+qd1*qd3*t20*1.59440520844656E-1+qd1*qd2*t22*3.853394307938084E-1-qd1*qd3*t21*1.023145577984553E-1+qd1*qd3*t22*5.311766694071189E-2-qd1*qd2*t24*1.218223287381895E-1-qd1*qd2*t36*1.889694424091543E-1-qd1*qd3*t38*1.176459415742204E-1-q3*t6*t11*2.547021474810983E-1+q3*t6*t14*1.678621322558363E-1+q3*qd1*qd2*t17*3.188810416893121E-1+q3*qd1*qd3*t17*2.857253974817815E-1+q3*qd1*qd2*t20*2.344461772182097E-1+q3*qd1*qd2*t21*1.062353338814238E-1+q3*qd1*qd2*t22*2.046291155969106E-1+q3*qd1*qd3*t21*7.843062771614694E-1+q3*qd1*qd2*t24*2.352918831484408E-1-qd1*qd2*t20*t23*2.857253974817815E-1-qd1*qd2*t22*t23*7.843062771614694E-1;
  A0[1][0] = t8*(-9.948823513618807E-1)+t9*9.948823513618807E-1-t26+t28+t29+t30*6.483473461732902E-1-t31-t32*6.415601204027139E-1+t34-t37-qd2*qd3*4.390752928151203E-1+q3*t7*1.161870910921767-q3*t10*1.161870910921767-t11*t33*9.448472120457717E-2+t17*t33*1.100521016723752E-1+t19*t33*7.967650041106784E-3+t20*t33*2.171948720100647E-1+t21*t33*9.986504377593244E-3-t22*t33*1.926697153969042E-1+t24*t33*6.091116436909474E-2+t33*t36*9.448472120457717E-2+q3*qd2*qd3*2.140063349286502-qd2*qd3*t38*2.352918831484408E-1-q3*t17*t33*1.59440520844656E-1-q3*t20*t33*1.172230886091048E-1-q3*t21*t33*5.311766694071189E-2-q3*t22*t33*1.023145577984553E-1-q3*t24*t33*1.176459415742204E-1+t20*t23*t33*1.428626987408908E-1+t22*t23*t33*3.921531385807347E-1;
  A0[2][0] = t6*2.195376464075602E-1-t25*1.9215503790456+t27*1.161870910921767+t30*1.161870910921767-t32*1.9215503790456+t33*1.097688232037801E-1-q3*t6*1.070031674643251-q3*t33*5.350158373216254E-1+t6*t38*1.176459415742204E-1+t17*t33*5.861154430455242E-2+t19*t33*5.88229707871102E-2-t20*t33*7.972026042232802E-2+t21*t33*5.115727889922766E-2-t22*t33*2.655883347035595E-2+t33*t38*5.88229707871102E-2-q3*t17*t33*1.428626987408908E-1-q3*t21*t33*3.921531385807347E-1;
