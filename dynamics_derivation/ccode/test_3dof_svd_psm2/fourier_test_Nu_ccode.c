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
  t26 = t25*5.801270808625774E-2;
  t27 = t9*2.399278871861801E-2;
  t28 = cos(t3);
  t29 = cos(t4);
  t30 = t29*1.66476265394877E-1;
  t31 = cos(t5);
  t32 = t8*2.399278871861801E-2;
  t33 = qd1*qd1;
  t34 = q3*t8*1.468701935658341;
  t35 = q3*t9*1.468701935658341;
  t36 = q2-5.816747949334241E-1;
  t37 = sin(t36);
  t38 = cos(2.908373974667121E-1);
  A0[0][0] = t7*(-5.831859557704719E-1)-t10*5.831859557704719E-1+t26+t27-t28*1.66476265394877E-1+t30-t31*5.801270808625774E-2+t32+t34+t35+sin(q1-2.908373974667121E-1)*5.391383695301182E-1+sin(q1+2.908373974667121E-1)*5.391383695301182E-1+cos(q1)*1.401743375332546E-1-sin(q1)*1.029111015703935-qd1*qd3*4.426931000323602E-2-q3*t7*1.95687355314037-q3*t10*1.95687355314037-t6*t11*1.815552921319758E-4-t6*t13*5.587416948589441E-2+t6*t20*5.066527304003224E-3+qd2*qd3*sin(t12)*4.37173175068707E-1+q3*qd1*qd3*1.080701327112414+qd1*qd2*t11*3.300847160388479E-2-qd2*qd3*t11*8.444432192184921E-3+qd1*qd2*t16*6.980057560679787E-1+qd1*qd3*t16*8.923902078228031E-2-qd1*qd2*t18*7.103596908521356E-3+qd1*qd2*t19*7.048803815861504E-1-qd1*qd3*t18*8.992052667295965E-2-qd1*qd3*t19*4.093957138792383E-2-qd1*qd2*t21*1.409233102412951E-1-qd1*qd2*t22*1.179084411368073-qd1*qd3*t21*1.335083307855163E-1+qd1*qd3*t22*2.504242044455207E-1-qd1*qd2*t24*4.00524992356549E-2+qd1*qd2*t37*3.300847160388479E-2-qd1*qd3*t38*8.992052667295965E-2+q3*t6*t13*2.185865875343535E-1-q3*t6*t20*4.22221609609246E-3-q3*qd1*qd2*t16*8.187914277584765E-2+q3*qd1*qd3*t16*4.812311492926828E-1-q3*qd1*qd2*t19*1.784780415645606E-1+q3*qd1*qd2*t21*5.008484088910414E-1+q3*qd1*qd2*t22*2.670166615710327E-1+q3*qd1*qd3*t21*5.99470177819731E-1+q3*qd1*qd2*t24*1.798410533459193E-1-qd1*qd2*t19*t23*4.812311492926828E-1-qd1*qd2*t22*t23*5.99470177819731E-1;
  A0[1][0] = t7*5.831859557704719E-1-t10*5.831859557704719E-1-t26-t27-t28*1.66476265394877E-1-t30-t31*5.801270808625774E-2+t32+t34-t35-qd2*qd3*8.853862000647204E-2+q3*t7*1.95687355314037-q3*t10*1.95687355314037-t11*t33*1.650423580194239E-2-t16*t33*3.490028780339893E-1+t18*t33*3.551798454260678E-3-t19*t33*3.524401907930752E-1+t21*t33*7.046165512064754E-2+t22*t33*5.895422056840367E-1+t24*t33*2.002624961782745E-2-t33*t37*1.650423580194239E-2+q3*qd2*qd3*2.161402654224828-qd2*qd3*t38*1.798410533459193E-1+q3*t16*t33*4.093957138792383E-2+q3*t19*t33*8.923902078228031E-2-q3*t21*t33*2.504242044455207E-1-q3*t22*t33*1.335083307855163E-1-q3*t24*t33*8.992052667295965E-2+t19*t23*t33*2.406155746463414E-1+t22*t23*t33*2.997350889098655E-1;
  A0[2][0] = t6*4.426931000323602E-2-t25*1.468701935658341+t28*1.95687355314037+t29*1.95687355314037-t31*1.468701935658341+t33*2.213465500161801E-2-q3*t6*1.080701327112414-q3*t33*5.403506635562069E-1+t6*t38*8.992052667295965E-2-t16*t33*4.461951039114016E-2+t18*t33*4.496026333647982E-2+t19*t33*2.046978569396191E-2+t21*t33*6.675416539275817E-2-t22*t33*1.252121022227603E-1+t33*t38*4.496026333647982E-2-q3*t16*t33*2.406155746463414E-1-q3*t21*t33*2.997350889098655E-1;