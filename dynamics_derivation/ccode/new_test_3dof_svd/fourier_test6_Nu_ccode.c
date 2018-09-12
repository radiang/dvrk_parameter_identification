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
  t26 = t25*3.737329811397667E-1;
  t27 = t9*4.428005238203674E-1;
  t28 = cos(t3);
  t29 = t28*2.551923879023658E-1;
  t30 = cos(t4);
  t31 = cos(t5);
  t32 = t8*4.428005238203674E-1;
  t33 = qd1*qd1;
  t34 = q3*t8*1.035454704121192;
  t35 = q3*t9*1.035454704121192;
  t36 = q2-5.816747949334241E-1;
  t37 = sin(t36);
  t38 = cos(2.908373974667121E-1);
  A0[0][0] = t7*(-4.306392830514957E-1)-t10*4.306392830514957E-1+t26+t27+t29-t30*2.551923879023658E-1-t31*3.737329811397667E-1+t32+t34+t35-sin(q1-2.908373974667121E-1)*1.677371892245841E-1-sin(q1+2.908373974667121E-1)*1.677371892245841E-1-cos(q1)*3.640013530587231E-2+sin(q1)*3.427862631208979E-1-qd1*qd3*1.270383235722824E-1-q3*t7*9.817282531152214E-1-q3*t10*9.817282531152214E-1+t6*t11*9.08831739312411E-4+t6*t13*8.591987643145814E-2-t6*t20*1.648420053151773E-1+qd2*qd3*sin(t12)*9.632182884763322E-2+q3*qd1*qd3*6.64059582295148E-1-qd1*qd2*t11*1.026962383007658E-2+qd2*qd3*t11*4.227124368894935E-2+qd1*qd2*t16*7.127023897544902E-1+qd1*qd3*t16*6.770650985860141E-2-qd1*qd2*t18*4.57632221803796E-2+qd1*qd2*t19*7.304150171695865E-1-qd1*qd3*t18*6.339518596660357E-2+qd1*qd3*t19*6.275649539231649E-2-qd1*qd2*t21*7.749413787939134E-2-qd1*qd2*t22*1.200449406871806-qd1*qd3*t21*1.947448334308838E-1+qd1*qd3*t22*1.235986288935607E-1-qd1*qd2*t24*5.842345002926515E-2-qd1*qd2*t37*1.026962383007658E-2-qd1*qd3*t38*6.339518596660357E-2+q3*t6*t13*4.816091442381661E-2+q3*t6*t20*2.113562184447467E-2+q3*qd1*qd2*t16*1.25512990784633E-1+q3*qd1*qd3*t16*2.414250091844575E-1-q3*qd1*qd2*t19*1.354130197172028E-1+q3*qd1*qd2*t21*2.471972577871214E-1+q3*qd1*qd2*t22*3.894896668617677E-1+q3*qd1*qd3*t21*4.226345731106905E-1+q3*qd1*qd2*t24*1.267903719332071E-1-qd1*qd2*t19*t23*2.414250091844575E-1-qd1*qd2*t22*t23*4.226345731106905E-1;
  A0[1][0] = t7*4.306392830514957E-1-t10*4.306392830514957E-1-t26-t27+t29+t30*2.551923879023658E-1-t31*3.737329811397667E-1+t32+t34-t35-qd2*qd3*2.540766471445649E-1+q3*t7*9.817282531152214E-1-q3*t10*9.817282531152214E-1+t11*t33*5.134811915038289E-3-t16*t33*3.563511948772451E-1+t18*t33*2.28816110901898E-2-t19*t33*3.652075085847932E-1+t21*t33*3.874706893969567E-2+t22*t33*6.00224703435903E-1+t24*t33*2.921172501463258E-2+t33*t37*5.134811915038289E-3+q3*qd2*qd3*1.328119164590296-qd2*qd3*t38*1.267903719332071E-1-q3*t16*t33*6.275649539231649E-2+q3*t19*t33*6.770650985860141E-2-q3*t21*t33*1.235986288935607E-1-q3*t22*t33*1.947448334308838E-1-q3*t24*t33*6.339518596660357E-2+t19*t23*t33*1.207125045922287E-1+t22*t23*t33*2.113172865553452E-1;
  A0[2][0] = t6*1.270383235722824E-1-t25*1.035454704121192+t28*9.817282531152214E-1+t30*9.817282531152214E-1-t31*1.035454704121192+t33*6.351916178614121E-2-q3*t6*6.64059582295148E-1-q3*t33*3.32029791147574E-1+t6*t38*6.339518596660357E-2-t16*t33*3.385325492930071E-2+t18*t33*3.169759298330179E-2-t19*t33*3.137824769615825E-2+t21*t33*9.737241671544192E-2-t22*t33*6.179931444678034E-2+t33*t38*3.169759298330179E-2-q3*t16*t33*1.207125045922287E-1-q3*t21*t33*2.113172865553452E-1;