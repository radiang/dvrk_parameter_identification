function [u] = impedence_control(ctrl,joint,init)

y = inv(ctrl.Ja) * inv(ctrl.Md)*(ctrl.Md*init.xd_dd + ctrl.Kd*(init.xd_d-ctrl.xe_d) + ctrl.Kp *(init.xd-ctrl.xe) - ctrl.Md*ctrl.Jd*joint.qdt(1:3)'-ctrl.ha);
y=simplify(y);

u = ctrl.D_t(1:3,1:3) * y + ctrl.Ja.'*ctrl.he;
u = simplify(u);

%u = subs(u,[ctrl.Fd.', ctrl.Xd.', ctrl.X0.'], [init.fd.',init.xdn.',init.x0n.']);

end