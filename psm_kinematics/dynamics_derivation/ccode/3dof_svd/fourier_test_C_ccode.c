  /*
  V1.2
  */
  t2 = q2*2.0;
  t3 = q3+t2;
  t4 = t2-5.816747949334241E-1;
  t5 = t2-2.908373974667121E-1;
  t6 = q3*2.0;
  t7 = -q3+t2;
  t8 = t2-t6;
  t9 = t2+t6;
  t10 = q3-t2+2.908373974667121E-1;
  t11 = q3+t2-2.908373974667121E-1;
  t12 = q3-2.908373974667121E-1;
  t13 = q3+2.908373974667121E-1;
  t14 = cos(t2);
  t15 = cos(t6);
  t16 = sin(t2);
  t17 = sin(t6);
  t18 = cos(t7);
  t19 = cos(t8);
  t20 = cos(t9);
  t21 = cos(t4);
  t22 = sin(t7);
  t23 = sin(t8);
  t24 = sin(t9);
  t25 = cos(t10);
  t26 = cos(t11);
  t27 = sin(t10);
  t28 = sin(t11);
  t29 = cos(t12);
  t30 = cos(t13);
  t31 = q2-5.816747949334241E-1;
  t32 = sin(t12);
  t33 = sin(t13);
  t34 = cos(q3);
  t35 = sin(q3);
  t36 = q3*q3;
  t37 = cos(t3);
  t38 = cos(t5);
  t39 = sin(t3);
  t40 = sin(t4);
  t41 = sin(t5);
  t42 = q2-q3;
  t43 = q2-t6;
  t44 = q2+t6;
  t45 = q2+q3-2.908373974667121E-1;
  t46 = -q2+q3+2.908373974667121E-1;
  t47 = q2+q3;
  t48 = q2-2.908373974667121E-1;
  t49 = cos(t46);
  t50 = sin(t46);
  t51 = cos(t47);
  t52 = cos(t48);
  t53 = sin(t47);
  t54 = sin(t48);
  t55 = sin(t31);
  t56 = cos(q2);
  t57 = sin(q2);
  t58 = cos(t42);
  t59 = cos(t43);
  t60 = cos(t44);
  t61 = sin(t42);
  t62 = sin(t43);
  t63 = sin(t44);
  t64 = cos(t45);
  t65 = sin(t45);
  t66 = t49*1.882831077710977E-4;
  t67 = t50*8.929284668366714E-4;
  t68 = t51*2.187023801023081E-3;
  t69 = qd1*t20*9.641693446251764E-3;
  t70 = qd2*t60*2.382719144631059E-1;
  t71 = qd2*t62*1.928338689250353E-2;
  t72 = qd2*t63*1.928338689250353E-2;
  t73 = cos(2.908373974667121E-1);
  t74 = qd1*t16*4.659634886359594E-1;
  t75 = sin(2.908373974667121E-1);
  t76 = qd3*t56*5.811329365550027E-1;
  t77 = qd1*t57*2.825070207040501E-1;
  t78 = sin(5.816747949334241E-1);
  t79 = cos(5.816747949334241E-1);
  t80 = t56*t56;
  t81 = t73*t73;
  t82 = t34*t34;
  t83 = qd2*t17*4.765438289262118E-1;
  t84 = t35*t73*3.765662155421955E-4;
  t85 = q3*qd2*3.547726451423457E-2;
  t86 = qd2*t34*2.047782617278766E-3;
  t87 = qd1*t34*t57*1.954397302546342E-2;
  t88 = qd2*t35*t75*1.506264862168782E-3;
  t89 = qd1*t34*t56*t75*1.506264862168782E-3;
  t90 = qd1*t35*t56*t75*7.143427734693371E-3;
  A0[0][0] = q3*5.641684399813122E-2-qd3*1.128336879962624E-1+t14*1.164908721589898E-1-t15*5.956797861577647E-2+t16*3.78435941017915E-3-t17*4.820846723125882E-3+t18*4.465001762795682E-2-t19*2.978398930788824E-2-t20*2.978398930788824E-2-t21*2.014865689785166E-1-t22*2.698969455342773E-3+t23*2.410423361562941E-3-t24*2.410423361562941E-3+t25*8.929284668366714E-4-t26*8.929284668366714E-4-t27*1.882831077710977E-4+t28*1.882831077710977E-4+t29*8.929284668366714E-4-t30*8.929284668366714E-4-t32*1.882831077710977E-4+t33*1.882831077710977E-4-t34*1.079489817887627E-4-t35*5.119456543196916E-4-t36*4.434658064279322E-3-t37*4.454206864616806E-2-t38*1.32012537564389E-2-t39*2.187023801023081E-3+t40*8.799769460551694E-2+t41*1.862739453565847E-3+t56*1.412535103520251E-1-t73*1.32012537564389E-2+t75*1.862739453565847E-3-t78*7.206792799878022E-2+cos(t31)*1.412535103520251E-1+q3*qd3*1.773863225711729E-2+q3*t21*5.641684399813122E-2-q3*t40*1.979388287369336E-2-qd2*t14*1.51374376407166E-2+qd2*t16*4.659634886359594E-1+qd3*t15*1.928338689250353E-2+qd2*t18*1.079587782137109E-2-qd3*t17*2.382719144631059E-1-qd2*t19*9.641693446251764E-3-qd3*t18*5.397938910685546E-3+qd2*t20*9.641693446251764E-3+qd3*t19*9.641693446251764E-3-qd2*t21*3.519907784220678E-1+qd3*t20*9.641693446251764E-3+qd2*t22*1.786000705118273E-1-qd3*t21*1.128336879962624E-1-qd2*t23*1.191359572315529E-1-qd3*t22*8.930003525591364E-2-qd2*t24*1.191359572315529E-1+qd3*t23*1.191359572315529E-1-qd2*t25*7.53132431084391E-4-qd3*t24*1.191359572315529E-1-qd2*t26*7.53132431084391E-4+qd3*t25*3.765662155421955E-4-qd2*t27*3.571713867346686E-3-qd3*t26*3.765662155421955E-4-qd2*t28*3.571713867346686E-3+qd3*t27*1.785856933673343E-3-qd3*t28*1.785856933673343E-3+qd3*t29*3.765662155421955E-4-qd3*t30*3.765662155421955E-4+qd3*t32*1.785856933673343E-3-qd3*t33*1.785856933673343E-3+qd3*t34*1.023891308639383E-3-qd3*t35*2.158979635775254E-4+qd2*t37*8.748095204092325E-3-qd2*t38*7.450957814263388E-3+qd3*t37*4.374047602046162E-3-qd2*t39*1.781682745846722E-1-qd2*t40*8.059462759140663E-1-qd3*t39*8.908413729233612E-2-qd2*t41*5.280501502575559E-2+qd3*t40*3.958776574738673E-2+qd2*t55*2.825070207040501E-1+qd2*t57*2.825070207040501E-1-t21*t36*4.434658064279322E-3+q3*qd2*t21*7.917553149477345E-2+q3*qd3*t21*1.773863225711729E-2+q3*qd2*t40*2.256673759925249E-1-qd2*t36*t40*1.773863225711729E-2-3.323437414950833E-1;
  A0[0][1] = t52*4.915806880307994E-2-t53*4.454206864616806E-2+t54*1.320771962680341E-1-t58*2.698969455342773E-3+t59*4.820846723125882E-3+t60*4.820846723125882E-3-t61*4.465001762795682E-2+t62*5.956797861577647E-2-t63*5.956797861577647E-2-t64*1.882831077710977E-4-t65*8.929284668366714E-4+t66+t67+t68+t69+t70+t71+t72+t74+t76+t77+q3*t54*1.266821036021037E-1-qd1*t14*1.51374376407166E-2+qd1*t18*1.079587782137109E-2-qd1*t19*9.641693446251764E-3-qd1*t21*3.519907784220678E-1+qd1*t22*1.786000705118273E-1-qd1*t23*1.191359572315529E-1-qd1*t24*1.191359572315529E-1-qd1*t25*7.53132431084391E-4-qd1*t26*7.53132431084391E-4-qd1*t27*3.571713867346686E-3-qd1*t28*3.571713867346686E-3+qd1*t37*8.748095204092325E-3-qd1*t38*7.450957814263388E-3-qd1*t39*1.781682745846722E-1-qd1*t40*8.059462759140663E-1-qd1*t41*5.280501502575559E-2+qd2*t49*3.571713867346686E-3-qd2*t50*7.53132431084391E-4+qd2*t51*1.781682745846722E-1-qd2*t52*5.283087850721363E-1+qd2*t53*8.748095204092325E-3+qd1*t55*2.825070207040501E-1+qd2*t54*1.966322752123198E-1-qd3*t54*5.06728414408415E-1+qd2*t58*1.786000705118273E-1-qd2*t59*2.382719144631059E-1+qd3*t59*2.382719144631059E-1-qd2*t61*1.079587782137109E-2+qd3*t60*2.382719144631059E-1-qd3*t62*1.928338689250353E-2+qd2*t64*3.571713867346686E-3+qd3*t63*1.928338689250353E-2-qd2*t65*7.53132431084391E-4+q3*qd1*t21*7.917553149477345E-2+q3*qd1*t40*2.256673759925249E-1-q3*qd2*t52*5.06728414408415E-1-qd1*t36*t40*1.773863225711729E-2-1.374393583700811E-2;
  A0[0][2] = qd1*(-1.128336879962624E-1)-t52*1.266821036021037E-1+t53*4.454206864616806E-2-t57*2.905664682775014E-1-t58*2.698969455342773E-3-t61*4.465001762795682E-2+t64*1.882831077710977E-4+t65*8.929284668366714E-4+t66+t67-t68+t69+t70-t71+t72+q3*qd1*1.773863225711729E-2+qd1*t15*1.928338689250353E-2-qd1*t17*2.382719144631059E-1-qd1*t18*5.397938910685546E-3+qd1*t19*9.641693446251764E-3-qd1*t21*1.128336879962624E-1-qd1*t22*8.930003525591364E-2+qd1*t23*1.191359572315529E-1-qd1*t24*1.191359572315529E-1+qd1*t25*3.765662155421955E-4-qd1*t26*3.765662155421955E-4+qd1*t27*1.785856933673343E-3-qd1*t28*1.785856933673343E-3+qd1*t29*3.765662155421955E-4-qd1*t30*3.765662155421955E-4+qd1*t32*1.785856933673343E-3-qd1*t33*1.785856933673343E-3+qd1*t34*1.023891308639383E-3-qd1*t35*2.158979635775254E-4+qd1*t37*4.374047602046162E-3-qd1*t39*8.908413729233612E-2+qd1*t40*3.958776574738673E-2-qd3*t49*3.571713867346686E-3+qd3*t50*7.53132431084391E-4-qd3*t51*1.781682745846722E-1-qd2*t54*5.06728414408415E-1-qd3*t53*8.748095204092325E-3+qd2*t56*5.811329365550027E-1+qd2*t59*2.382719144631059E-1-qd3*t58*1.786000705118273E-1+qd3*t61*1.079587782137109E-2-qd3*t64*3.571713867346686E-3+qd3*t65*7.53132431084391E-4+q3*qd1*t21*1.773863225711729E-2;
  A0[1][0] = -t74-t76-t77+qd1*t14*1.51374376407166E-2+t15*t56*9.641693446251764E-3-t17*t56*1.191359572315529E-1-t34*t56*5.119456543196916E-4-t34*t57*8.919208627412488E-2+t35*t56*1.079489817887627E-4-t35*t57*4.885993256365854E-3+t56*t73*4.915806880307994E-2+t57*t73*1.320771962680341E-1-t56*t75*1.320771962680341E-1+t57*t75*4.915806880307994E-2+q3*t57*t73*1.266821036021037E-1-q3*t56*t75*1.266821036021037E-1+qd1*t15*t16*2.382719144631059E-1+qd1*t16*t17*1.928338689250353E-2-qd1*t14*t34*1.954397302546342E-2+qd1*t14*t35*3.567683450964995E-1-qd1*t16*t34*4.317959271550508E-4-qd1*t16*t35*2.047782617278766E-3+qd3*t15*t56*4.765438289262118E-1+qd3*t17*t56*3.856677378500706E-2+qd1*t14*t73*7.450957814263388E-3-qd1*t14*t75*5.280501502575559E-2+qd1*t16*t73*5.280501502575559E-2+qd1*t16*t75*7.450957814263388E-3-qd1*t14*t78*8.059462759140663E-1-qd3*t34*t56*4.317959271550508E-4+qd1*t14*t79*3.519907784220678E-1+qd3*t34*t57*1.954397302546342E-2-qd3*t35*t56*2.047782617278766E-3+qd1*t16*t78*3.519907784220678E-1-qd3*t35*t57*3.567683450964995E-1+qd1*t16*t79*8.059462759140663E-1+qd1*t56*t78*2.825070207040501E-1-qd1*t57*t79*2.825070207040501E-1-t34*t57*t73*1.785856933673343E-3+t34*t56*t75*1.785856933673343E-3+t35*t57*t73*3.765662155421955E-4-t35*t56*t75*3.765662155421955E-4+q3*qd1*t14*t78*2.256673759925249E-1-q3*qd1*t14*t79*7.917553149477345E-2-q3*qd1*t16*t78*7.917553149477345E-2-q3*qd1*t16*t79*2.256673759925249E-1+qd1*t14*t34*t73*1.506264862168782E-3+qd1*t14*t35*t73*7.143427734693371E-3+qd1*t16*t34*t75*1.506264862168782E-3+qd1*t16*t35*t75*7.143427734693371E-3-qd1*t14*t36*t78*1.773863225711729E-2+qd1*t16*t36*t79*1.773863225711729E-2-qd3*t34*t57*t73*1.506264862168782E-3+qd3*t34*t56*t75*1.506264862168782E-3-qd3*t35*t57*t73*7.143427734693371E-3+qd3*t35*t56*t75*7.143427734693371E-3-1.374393583700811E-2;
  A0[1][1] = q3*1.128336879962624E-1-qd3*2.256673759925249E-1+t15*1.191359572315529E-1+t17*9.641693446251764E-3-t34*2.158979635775254E-4-t35*1.023891308639383E-3-t36*8.869316128558643E-3-t73*2.640250751287779E-2+t75*3.725478907131694E-3+q3*qd3*3.547726451423457E-2-qd3*t15*3.856677378500706E-2+qd3*t17*4.765438289262118E-1+qd3*t34*2.047782617278766E-3-qd3*t35*4.317959271550508E-4+t34*t75*7.53132431084391E-4+t35*t75*3.571713867346686E-3-qd3*t34*t75*7.143427734693371E-3+qd3*t35*t75*1.506264862168782E-3-4.711246955561407E-1;
  A0[1][2] = qd2*(-2.256673759925249E-1)-t34*8.919208627412488E-2-t35*4.885993256365854E-3+t83+t84+t85+t86+t87+t88+t89+t90-qd2*t15*3.856677378500706E-2-qd2*t35*4.317959271550508E-4+qd3*t34*1.954397302546342E-2-qd3*t35*3.567683450964995E-1-qd1*t56*5.811329365550027E-1-t34*t73*1.785856933673343E-3+qd1*t15*t56*4.765438289262118E-1+qd1*t17*t56*3.856677378500706E-2-qd1*t34*t56*4.317959271550508E-4-qd1*t35*t56*2.047782617278766E-3-qd1*t35*t57*3.567683450964995E-1-qd3*t34*t73*1.506264862168782E-3-qd2*t34*t75*7.143427734693371E-3-qd3*t35*t73*7.143427734693371E-3-qd1*t34*t57*t73*1.506264862168782E-3-qd1*t35*t57*t73*7.143427734693371E-3-1.979388287369336E-2;
  A0[2][0] = qd1*2.256673759925249E-1-t57*2.905664682775014E-1-q3*qd1*3.547726451423457E-2+qd1*t16*3.958776574738673E-2-qd1*t34*2.047782617278766E-3+qd1*t35*4.317959271550508E-4+qd2*t56*1.057676765481214-qd1*t78*3.958776574738673E-2-qd1*t80*1.871006022075178E-1-qd1*t81*2.256673759925249E-1-t34*t56*4.885993256365854E-3-t34*t57*1.079489817887627E-4+t35*t56*8.919208627412488E-2-t35*t57*5.119456543196916E-4-t56*t73*1.266821036021037E-1-t57*t75*1.266821036021037E-1+q3*qd1*t80*3.547726451423457E-2+q3*qd1*t81*3.547726451423457E-2+qd2*t34*t56*4.317959271550508E-4-qd2*t34*t57*1.954397302546342E-2+qd2*t35*t56*2.047782617278766E-3+qd2*t35*t57*3.567683450964995E-1+qd1*t34*t75*7.143427734693371E-3-qd1*t35*t75*1.506264862168782E-3+qd1*t34*t80*2.047782617278766E-3-qd1*t35*t80*4.317959271550508E-4-qd2*t56*t82*9.530876578524235E-1+qd1*t80*t81*4.513347519850498E-1-qd1*t80*t82*7.713354757001412E-2+t34*t56*t73*3.765662155421955E-4+t35*t56*t73*1.785856933673343E-3+t34*t57*t75*3.765662155421955E-4+t35*t57*t75*1.785856933673343E-3-q3*qd1*t80*t81*7.095452902846915E-2-qd2*t34*t35*t56*7.713354757001412E-2+qd1*t34*t56*t57*3.567683450964995E-1+qd1*t35*t56*t57*1.954397302546342E-2+qd1*t34*t35*t80*9.530876578524235E-1+qd2*t34*t57*t73*1.506264862168782E-3-qd2*t34*t56*t75*1.506264862168782E-3+qd2*t35*t57*t73*7.143427734693371E-3-qd2*t35*t56*t75*7.143427734693371E-3-qd1*t34*t75*t80*7.143427734693371E-3+qd1*t35*t75*t80*1.506264862168782E-3-qd1*t56*t57*t81*1.583510629895469E-1+qd1*t73*t75*t80*1.583510629895469E-1+qd1*t34*t56*t57*t73*7.143427734693371E-3-qd1*t35*t56*t57*t73*1.506264862168782E-3+qd1*t56*t57*t73*t75*4.513347519850498E-1-q3*qd1*t56*t57*t73*t75*7.095452902846915E-2;
  A0[2][1] = qd2*1.871006022075178E-1-t34*8.919208627412488E-2-t35*4.885993256365854E-3-t83+t84-t85-t86-t87-t88-t89-t90+qd2*t35*4.317959271550508E-4+qd1*t56*1.057676765481214+qd2*t82*7.713354757001412E-2-t34*t73*1.785856933673343E-3+qd1*t34*t56*4.317959271550508E-4+qd1*t35*t56*2.047782617278766E-3+qd1*t35*t57*3.567683450964995E-1+qd2*t34*t75*7.143427734693371E-3-qd1*t56*t82*9.530876578524235E-1-qd1*t34*t35*t56*7.713354757001412E-2+qd1*t34*t57*t73*1.506264862168782E-3+qd1*t35*t57*t73*7.143427734693371E-3-1.979388287369336E-2;
  A0[2][2] = 1.342936099164326;