function T = DH(thet, a , d, alpha)
    T = [cos(thet), -sin(thet)*cos(alpha), sin(thet)*sin(alpha), a*cos(thet); 
           sin(thet), cos(thet)*cos(alpha), -cos(thet)*sin(alpha), a*sin(thet);
           0, sin(alpha), cos(alpha), d;
           0, 0, 0, 1];
       
       