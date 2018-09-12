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
  t26 = t9*2.032279836128066E-1;
  t27 = cos(t3);
  t28 = cos(t4);
  t29 = t28*9.007271089304384E-1;
  t30 = cos(t5);
  t31 = t30*5.270941832797361E-1;
  t32 = t8*2.032279836128066E-1;
  t33 = qd1*qd1;
  t34 = q3*t8*2.037232783071054;
  t35 = q3*t9*2.037232783071054;
  t36 = q2-5.816747949334241E-1;
  t37 = sin(t36);
  t38 = cos(2.908373974667121E-1);
  A0[0][0] = t7*(-7.79352978893546E-1)-t10*7.79352978893546E-1-t25*5.270941832797361E-1+t26-t27*9.007271089304384E-1+t29+t31+t32+t34+t35+sin(q1-2.908373974667121E-1)*1.996998990940265+sin(q1+2.908373974667121E-1)*1.996998990940265+cos(q1)*1.868657398736351E-1+sin(q1)*1.436840152437411-qd1*qd3*2.349660384684137E-1-q3*t7*2.134569724011746-q3*t10*2.134569724011746+t6*t11*9.087872029043785E-3-t6*t13*1.465161046079982E-1+t6*t20*1.38095129637193E-1-qd2*qd3*sin(t12)*6.567738420853454E-1+q3*qd1*qd3*1.356453486444135+qd1*qd2*t11*1.222652443432815E-1+qd2*qd3*t11*4.226917222811063E-1+qd1*qd2*t16*6.799345763180272E-1+qd1*qd3*t16*1.165082681687E-1+qd1*qd2*t18*6.454214489139626E-2+qd1*qd2*t19*5.529776955925479E-1-qd1*qd3*t18*1.247285377390441E-1-qd1*qd3*t19*2.215053400533017E-1-qd1*qd2*t21*2.663779650220473E-1-qd1*qd2*t22*1.1678987210701-qd1*qd3*t21*3.514743066371136E-1+qd1*qd3*t22*4.455978859867352E-1-qd1*qd2*t24*1.054422919911341E-1+qd1*qd2*t37*1.222652443432815E-1-qd1*qd3*t38*1.247285377390441E-1-q3*t6*t13*3.283869210426727E-1+q3*t6*t20*2.113458611405531E-1-q3*qd1*qd2*t16*4.430106801066034E-1+q3*qd1*qd3*t16*5.249299015171742E-1-q3*qd1*qd2*t19*2.330165363374E-1+q3*qd1*qd2*t21*8.911957719734704E-1+q3*qd1*qd2*t22*7.029486132742273E-1+q3*qd1*qd3*t21*8.315235849269608E-1+q3*qd1*qd2*t24*2.494570754780882E-1-qd1*qd2*t19*t23*5.249299015171742E-1-qd1*qd2*t22*t23*8.315235849269608E-1;
  A0[1][0] = t7*7.79352978893546E-1-t10*7.79352978893546E-1+t25*5.270941832797361E-1-t26-t27*9.007271089304384E-1-t29+t31+t32+t34-t35-qd2*qd3*4.699320769368273E-1+q3*t7*2.134569724011746-q3*t10*2.134569724011746-t11*t33*6.113262217164076E-2-t16*t33*3.399672881590136E-1-t18*t33*3.227107244569813E-2-t19*t33*2.764888477962739E-1+t21*t33*1.331889825110236E-1+t22*t33*5.8394936053505E-1+t24*t33*5.272114599556705E-2-t33*t37*6.113262217164076E-2+q3*qd2*qd3*2.71290697288827-qd2*qd3*t38*2.494570754780882E-1+q3*t16*t33*2.215053400533017E-1+q3*t19*t33*1.165082681687E-1-q3*t21*t33*4.455978859867352E-1-q3*t22*t33*3.514743066371136E-1-q3*t24*t33*1.247285377390441E-1+t19*t23*t33*2.624649507585871E-1+t22*t23*t33*4.157617924634804E-1;
  A0[2][0] = t6*2.349660384684137E-1-t25*2.037232783071054+t27*2.134569724011746+t28*2.134569724011746-t30*2.037232783071054+t33*1.174830192342068E-1-q3*t6*1.356453486444135-q3*t33*6.782267432220675E-1+t6*t38*1.247285377390441E-1-t16*t33*5.825413408435E-2+t18*t33*6.236426886952206E-2+t19*t33*1.107526700266508E-1+t21*t33*1.757371533185568E-1-t22*t33*2.227989429933676E-1+t33*t38*6.236426886952206E-2-q3*t16*t33*2.624649507585871E-1-q3*t21*t33*4.157617924634804E-1;