  /*
  V1.2
  */
  t2 = (qd3/fabs(qd3));
  t3 = fabs(qd3);
  t4 = t3*1.818181818181818E2;
  t5 = pow(t4,3.0/5.0);
  t6 = exp(-t5);
  A0[0][0] = q1*(-3.377887812144361)+qd1*9.762974640867822E-2+((qd1/fabs(qd1)))*6.596858591952005E-2;
  A0[1][0] = q2*1.241395313148135+qd2*1.201384746374288E-1+((qd2/fabs(qd2)))*1.169917826568807E-1;
  A0[2][0] = qd3*5.700194022005638E-1-t2*t6*3.133301898199931E-1-t2*(t6-1.0)*6.588271442564433E-1+1.314823716676224E-1;
