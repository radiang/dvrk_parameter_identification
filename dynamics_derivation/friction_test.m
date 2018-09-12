syms v x % Joint Angles 

v = linspace(-0.3,0.3,5000); 

deadband = 0.005; %m/s
pos_deadband = 0.0001; %m

Fs_p = 0.6;
Fs_n = -0.4;
for i = 1:length(v)
    qd3   = v(i); 
    %Fr(i) = (2856832636712695*qd3)/2251799813685248 + 4912656234329547/(4503599627370496*(exp(-400*qd3) + 1)) - 4912656234329547/9007199254740992;
    if( pos_deadband<v(i)&& v(i)<deadband)
        Fr(i) = Fs_p;
    elseif(-deadband<v(i)&& v(i)<-pos_deadband)
        Fr(i) = Fs_n;
    elseif(abs(v(i))<pos_deadband)
        Fr(i) = 0;
    else
        Fr(i) = 1.3*qd3 + 1.1/(exp(-400.0*qd3) + 1.0) - 0.55;
    end
end

plot(v,Fr);
s = 0.01;
xlim([-s, s]);
