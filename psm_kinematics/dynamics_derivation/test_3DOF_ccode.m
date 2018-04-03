clear all 
close all

load('data/lcg3_all.mat');


J_end_t = subs(J_end, symvar(J_end), [q1t(t) q2t(t) q3t(t) 0 0 0]);
J_diff_t = diff(J_end_t,t);
