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
  t9 = cos(2.908373974667121E-1);
  t10 = cos(t3);
  t11 = cos(q2);
  t12 = q2-2.908373974667121E-1;
  t13 = sin(t12);
  t14 = sin(q2);
  t15 = t13*3.428948186011343E-2;
  t16 = t11*3.973047675957156E-2;
  t17 = q3*t14*1.962732185945708E-1;
  t18 = t14*(-5.654631427709586E-2)+t15+t16+t17-q3*t13*7.517197431331207E-2-2.768901199983599E-2;
  t19 = sin(2.908373974667121E-1);
  t20 = cos(t12);
  t21 = t20*7.517197431331207E-2;
  t22 = t11*(-1.962732185945708E-1)+t21;
  t23 = t19*(-1.051618492336644E-2)-3.022086063547907E-2;
  A0[0][0] = q3*(-1.22637104529915E-1)+t4*3.04969818341196E-2-t5*1.969286973527256E-2+t7*7.887138692524833E-4+t8*1.958850817239807E-2+t9*3.467391647725919E-2+t10*3.467391647725919E-2+t11*5.652722315145276E-3+t19*2.760884265136983E-2+cos(q2-5.816747949334241E-1)*5.652722315145276E-3+sin(t3)*2.760884265136983E-2-q3*t4*2.311594431817279E-1+q3*t5*3.686751603505449E-2+q3*t7*1.085223386518129E-1-q3*t9*1.051618492336644E-2-q3*t10*1.051618492336644E-2+t4*t8*3.505394974455481E-2-t7*t8*1.546544157215675E-2-q3*sin(t6)*6.708837667053356E-2+2.149682017249759E-1;
  A0[0][1] = t18;
  A0[0][2] = t22;
  A0[1][0] = t18;
  A0[1][1] = q3*(-2.4527420905983E-1)+t8*3.917701634479613E-2+t9*6.934783295451838E-2+t19*5.521768530273965E-2-q3*t9*2.103236984673289E-2+3.73888599253754E-1;
  A0[1][2] = t23;
  A0[2][0] = t22;
  A0[2][1] = t23;
  A0[2][2] = 3.917701634479613E-2;