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
  t20 = qd2*qd2;
  t21 = q3*6.597818317799755E-1;
  A0[0][0] = t20*(t12*3.98768609581117E-4+t14*1.272488566997194E-1-t15*1.875069763910439E-1+q3*t14*6.403283040898087E-2+q3*t15*9.273688594909697E-3)+qd1*qd3*(t5*(-1.794782236896359E-1)+t6*1.259012635482498E-1+t7*6.46554509696853E-2+t9*5.055529072692916E-2-t11*6.305456736637622E-2-t18*6.305456736637622E-2+t21+q3*t5*4.203637824425081E-1+q3*t9*2.394180493374674E-1-1.289229329627067E-1)+qd2*qd3*(t12*1.854737718981939E-2+sin(t13)*1.280656608179617E-1)-qd1*qd2*(t5*7.756425000174515E-2+t6*1.207354047998776-t7*7.438567963111598E-1-t9*7.125710882044354E-1+t10*5.384346710689077E-2+t11*4.345590526044919E-2+t12*1.17291273346395E-2+t17*1.17291273346395E-2-q3*t5*2.518025270964995E-1-q3*t6*3.589564473792718E-1+q3*t7*1.011105814538583E-1-q3*t9*1.293109019393706E-1-q3*t10*1.261091347327524E-1+t6*t8*4.203637824425081E-1+t7*t8*2.394180493374674E-1);
  A0[1][0] = t19*(t5*3.878212500087258E-2+t6*6.036770239993881E-1-t7*3.719283981555799E-1-t9*3.562855441022177E-1+t10*2.692173355344539E-2+t11*2.17279526302246E-2+t12*5.86456366731975E-3+t17*5.86456366731975E-3-q3*t5*1.259012635482498E-1-q3*t6*1.794782236896359E-1+q3*t7*5.055529072692916E-2-q3*t9*6.46554509696853E-2-q3*t10*6.305456736637622E-2+t6*t8*2.101818912212541E-1+t7*t8*1.197090246687337E-1)-qd2*qd3*(q3*(-1.319563663559951)+t18*1.261091347327524E-1+2.578458659254135E-1);
  A0[2][0] = t20*(t18*6.305456736637622E-2-t21+1.289229329627067E-1)-t19*(q3*3.298909158899877E-1-t5*8.973911184481796E-2+t6*6.295063177412488E-2+t7*3.232772548484265E-2+t9*2.527764536346458E-2-t11*3.152728368318811E-2-t18*3.152728368318811E-2+q3*t5*2.101818912212541E-1+q3*t9*1.197090246687337E-1-6.446146648135337E-2);