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
  t15 = t14-2.908373974667121E-1;
  t16 = cos(t15);
  t17 = t14-5.816747949334241E-1;
  t18 = sin(t17);
  t19 = cos(q2);
  t20 = cos(t14);
  t21 = sin(t14);
  t22 = q3*q3;
  t23 = cos(t17);
  t24 = sin(t15);
  t25 = cos(t2);
  t26 = t25*1.243195369999645E-1;
  t27 = t9*2.499078731758794;
  t28 = cos(t3);
  t29 = cos(t4);
  t30 = t29*3.870385473063326E-1;
  t31 = cos(t5);
  t32 = t8*2.499078731758794;
  t33 = qd1*qd1;
  t34 = q3*t7*1.865078056348722;
  t35 = q2-5.816747949334241E-1;
  t36 = sin(t35);
  t37 = q3*t10*1.865078056348722;
  t38 = cos(2.908373974667121E-1);
  A0[0][0] = q1*(-3.238062172669773)-qd1*1.149084940876338E-1-t7*1.754668689629027-t10*1.754668689629027+t26+t27-t28*3.870385473063326E-1+t30-t31*1.243195369999645E-1+t32+t34+t37-sin(q1-2.908373974667121E-1)*1.715471252807633-sin(q1+2.908373974667121E-1)*1.715471252807633+cos(q1)*5.817604155962425E-2+sin(q1)*5.870613625075947+qd1*qd3*3.128855268898426E-1-q3*t8*1.928569842615533-q3*t9*1.928569842615533-t6*t11*2.754650838059389E-2-t6*t13*4.948908070138196E-2+t6*t19*1.31749016327031E-2+qd2*qd3*sin(t12)*1.143788413216312E-1-q3*qd1*qd3*1.548427713862961-qd1*qd2*t11*1.050288522127122E-1-qd2*qd3*t11*9.146061529123987E-2-qd1*qd2*t16*3.530559125042486E-1+qd1*qd3*t16*1.180757046499306E-1+qd1*qd2*t18*1.771135569748959E-2-qd1*qd3*t18*1.579749172678909E-1+qd1*qd2*t20*1.508759758283309E-1+qd1*qd2*t21*7.067009176328439E-1-qd1*qd3*t20*5.213814796494867E-1-qd1*qd3*t21*3.735130390553915E-1-qd1*qd2*t24*1.56414443894846E-1+qd1*qd3*t23*8.342670065393293E-1-qd1*qd2*t36*1.050288522127122E-1+qd1*qd3*t38*1.180757046499306E-1+q3*t6*t13*5.718942066081562E-2-q3*t6*t19*4.573030764561994E-2-q3*qd1*qd2*t18*1.668534013078659-q3*qd1*qd2*t20*7.470260781107831E-1+q3*qd1*qd2*t21*1.042762959298973-q3*qd1*qd3*t20*7.871713643328705E-1-q3*qd1*qd2*t23*3.159498345357817E-1-q3*qd1*qd2*t24*2.361514092998611E-1-q3*qd1*qd3*t23*7.612563495300906E-1+qd1*qd2*t18*t22*7.612563495300906E-1+qd1*qd2*t21*t22*7.871713643328705E-1;
  A0[1][0] = q2*(-1.254046488288379)-qd2*1.315568217850305E-1+t7*1.754668689629027-t10*1.754668689629027-t26-t27-t28*3.870385473063326E-1-t30-t31*1.243195369999645E-1+t32-t34+t37+qd2*qd3*6.257710537796852E-1-q3*t8*1.928569842615533+q3*t9*1.928569842615533+t11*t33*5.251442610635612E-2+t16*t33*1.765279562521243E-1-t18*t33*8.855677848744793E-3-t20*t33*7.543798791416547E-2-t21*t33*3.533504588164219E-1+t24*t33*7.8207221947423E-2+t33*t36*5.251442610635612E-2-q3*qd2*qd3*3.096855427725922+qd2*qd3*t38*2.361514092998611E-1+q3*t18*t33*8.342670065393293E-1+q3*t20*t33*3.735130390553915E-1-q3*t21*t33*5.213814796494867E-1+q3*t23*t33*1.579749172678909E-1+q3*t24*t33*1.180757046499306E-1-t18*t22*t33*3.806281747650453E-1-t21*t22*t33*3.935856821664352E-1;
  A0[2][0] = qd3*(-7.808391374623841E-1)-t6*3.128855268898426E-1+t25*1.928569842615533-t28*1.865078056348722-t29*1.865078056348722+t31*1.928569842615533-t33*1.564427634449213E-1+q3*t6*1.548427713862961+q3*t33*7.742138569314805E-1-t6*t38*1.180757046499306E-1-t16*t33*5.903785232496529E-2+t18*t33*7.898745863394543E-2+t20*t33*2.606907398247433E-1+t21*t33*1.867565195276958E-1-t23*t33*4.171335032696646E-1-t33*t38*5.903785232496529E-2+q3*t20*t33*3.935856821664352E-1+q3*t23*t33*3.806281747650453E-1;