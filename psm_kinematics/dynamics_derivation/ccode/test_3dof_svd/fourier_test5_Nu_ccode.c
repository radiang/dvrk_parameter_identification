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
  t26 = t25*6.484272618143839E-1;
  t27 = cos(t3);
  t28 = t27*9.523398220251548E-1;
  t29 = t10*1.058471470556645;
  t30 = cos(t4);
  t31 = t7*1.058471470556645;
  t32 = cos(t5);
  t33 = qd1*qd1;
  t34 = q3*t8*1.585305496100866;
  t35 = q3*t9*1.585305496100866;
  t36 = q2-5.816747949334241E-1;
  t37 = sin(t36);
  t38 = cos(2.908373974667121E-1);
  A0[0][0] = t8*(-2.580608029001201)-t9*2.580608029001201+t26+t28+t29-t30*9.523398220251548E-1+t31-t32*6.484272618143839E-1+t34+t35+sin(q1-2.908373974667121E-1)*8.40121250291409E-1+sin(q1+2.908373974667121E-1)*8.40121250291409E-1+cos(q1)*6.008653141803544E-3+sin(q1)*4.001556733758758-qd1*qd3*1.931562620271773E-1-q3*t7*1.635871276508999-q3*t10*1.635871276508999-t6*t11*1.084696263664992E-2+t6*t13*3.623573511402178E-1-t6*t20*3.022330207222726E-1+qd2*qd3*sin(t12)*3.544374730744005E-1+q3*qd1*qd3*1.049354261305117+qd1*qd2*t11*5.143599491580055E-2-qd2*qd3*t11*5.045098900767404E-1+qd1*qd2*t16*9.553974388905091E-1-qd1*qd3*t16*3.187759368246916E-1-qd1*qd2*t18*7.939925654870006E-2+qd1*qd2*t19*1.071731233950547-qd1*qd3*t18*9.705952016944075E-2+qd1*qd3*t19*2.341978541919003E-1+qd1*qd2*t21*5.352180068678806E-3-qd1*qd2*t22*1.773266356987585+qd1*qd3*t21*1.256196747975143E-1+qd1*qd3*t22*3.728418028481174E-2+qd1*qd2*t24*3.768590243925429E-2+qd1*qd2*t37*5.143599491580055E-2-qd1*qd3*t38*9.705952016944075E-2+q3*t6*t13*1.772187365372002E-1-q3*t6*t20*2.522549450383702E-1+q3*qd1*qd2*t16*4.683957083838006E-1+q3*qd1*qd3*t16*4.022907935088457E-1+q3*qd1*qd2*t19*6.375518736493831E-1+q3*qd1*qd2*t21*7.456836056962349E-2-q3*qd1*qd2*t22*2.512393495950286E-1+q3*qd1*qd3*t21*6.470634677962717E-1+q3*qd1*qd2*t24*1.941190403388815E-1-qd1*qd2*t19*t23*4.022907935088457E-1-qd1*qd2*t22*t23*6.470634677962717E-1;
  A0[1][0] = t8*(-2.580608029001201)+t9*2.580608029001201-t26+t28+t29+t30*9.523398220251548E-1-t31-t32*6.484272618143839E-1+t34-t35-qd2*qd3*3.863125240543546E-1+q3*t7*1.635871276508999-q3*t10*1.635871276508999-t11*t33*2.571799745790028E-2-t16*t33*4.776987194452545E-1+t18*t33*3.969962827435003E-2-t19*t33*5.358656169752733E-1-t21*t33*2.676090034339403E-3+t22*t33*8.866331784937926E-1-t24*t33*1.884295121962714E-2-t33*t37*2.571799745790028E-2+q3*qd2*qd3*2.098708522610235-qd2*qd3*t38*1.941190403388815E-1-q3*t16*t33*2.341978541919003E-1-q3*t19*t33*3.187759368246916E-1-q3*t21*t33*3.728418028481174E-2+q3*t22*t33*1.256196747975143E-1-q3*t24*t33*9.705952016944075E-2+t19*t23*t33*2.011453967544229E-1+t22*t23*t33*3.235317338981358E-1;
  A0[2][0] = t6*1.931562620271773E-1-t25*1.585305496100866+t27*1.635871276508999+t30*1.635871276508999-t32*1.585305496100866+t33*9.657813101358865E-2-q3*t6*1.049354261305117-q3*t33*5.246771306525587E-1+t6*t38*9.705952016944075E-2+t16*t33*1.593879684123458E-1+t18*t33*4.852976008472037E-2-t19*t33*1.170989270959502E-1-t21*t33*6.280983739875714E-2-t22*t33*1.864209014240587E-2+t33*t38*4.852976008472037E-2-q3*t16*t33*2.011453967544229E-1-q3*t21*t33*3.235317338981358E-1;