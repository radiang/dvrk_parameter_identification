  /*
  V1.2
  */
  t2 = (qd3/fabs(qd3));
  t3 = fabs(qd3);
  t4 = t3*1.818181818181818E2;
  t5 = pow(t4,3.0/5.0);
  t6 = exp(-t5);
  A0[0][0] = q1*5.152071075440072E-1+qd1*9.243722677981868E-2+((qd1/fabs(qd1)))*6.087746855471963E-2;
  A0[1][0] = q2*9.290938179944848E-1+qd2*1.180489647496518E-1+((qd2/fabs(qd2)))*1.298683232087144E-1;
  A0[2][0] = qd3*2.807506303910837+t2*t6*1.044713629294216-t2*(t6-1.0)*1.123275372913279E-1+8.747920427688542E-1;
