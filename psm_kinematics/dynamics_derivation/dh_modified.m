function T = dh_modified(thet, a , d, alpha)
    
    T = [cos(thet), -sin(thet), sin(thet), a; 
           sin(thet)*cos(alpha), cos(thet)*cos(alpha), -sin(alpha), -d*sin(alpha);
           sin(thet)*sin(alpha), cos(thet)*sin(alpha), cos(alpha), d*cos(alpha);
           0, 0, 0, 1];
       
       