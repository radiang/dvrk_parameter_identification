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
  t19 = qd1*qd1;
  t20 = q3*1.356453486444135;
  t21 = qd2*qd2;
  A0[0][0] = t21*(t12*9.087872029043785E-3-t14*1.465161046079982E-1+t15*1.38095129637193E-1-q3*t14*3.283869210426727E-1+q3*t15*2.113458611405531E-1)+qd1*qd2*(t5*(-2.663779650220473E-1)-t6*1.1678987210701+t7*5.529776955925479E-1+t9*6.799345763180272E-1-t10*1.054422919911341E-1+t11*6.454214489139626E-2+t12*1.222652443432815E-1+t17*1.222652443432815E-1+q3*t5*8.911957719734704E-1+q3*t6*7.029486132742273E-1-q3*t7*2.330165363374E-1-q3*t9*4.430106801066034E-1+q3*t10*2.494570754780882E-1-t6*t8*8.315235849269608E-1-t7*t8*5.249299015171742E-1)+qd2*qd3*(t12*4.226917222811063E-1-sin(t13)*6.567738420853454E-1)-qd1*qd3*(t5*3.514743066371136E-1-t6*4.455978859867352E-1+t7*2.215053400533017E-1-t9*1.165082681687E-1+t11*1.247285377390441E-1+t18*1.247285377390441E-1-t20-q3*t5*8.315235849269608E-1-q3*t9*5.249299015171742E-1+2.349660384684137E-1);
  A0[1][0] = -t19*(t5*(-1.331889825110236E-1)-t6*5.8394936053505E-1+t7*2.764888477962739E-1+t9*3.399672881590136E-1-t10*5.272114599556705E-2+t11*3.227107244569813E-2+t12*6.113262217164076E-2+t17*6.113262217164076E-2+q3*t5*4.455978859867352E-1+q3*t6*3.514743066371136E-1-q3*t7*1.165082681687E-1-q3*t9*2.215053400533017E-1+q3*t10*1.247285377390441E-1-t6*t8*4.157617924634804E-1-t7*t8*2.624649507585871E-1)-qd2*qd3*(q3*(-2.71290697288827)+t18*2.494570754780882E-1+4.699320769368273E-1);
  A0[2][0] = -t19*(q3*6.782267432220675E-1-t5*1.757371533185568E-1+t6*2.227989429933676E-1-t7*1.107526700266508E-1+t9*5.825413408435E-2-t11*6.236426886952206E-2-t18*6.236426886952206E-2+q3*t5*4.157617924634804E-1+q3*t9*2.624649507585871E-1-1.174830192342068E-1)+t21*(t18*1.247285377390441E-1-t20+2.349660384684137E-1);