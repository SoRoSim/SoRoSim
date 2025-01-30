function xci=piecewise_logmap(g)

theta = acos(trace(g)/2-1);
gp2=g*g;
gp3=gp2*g;

if theta<1e-2
    
    f1 = -11/6;
    f2 = 3;
    f3 = -3/2;
    f4 = 1/3;

    xcihat      =f1*diag([1 1 1 1])+f2*g+f3*gp2+f4*gp3;
else
    
    t0=theta;
    t1=sin(t0);
    t2=cos(t0);
    t3=2*t1*t2;
    t4=1-2*t1^2;
    t5=t0*t4;
    
    xcihat      = 0.125*(csc(t0/2)^3)*sec(t0/2)...
                  *((t5-t1)*diag([1 1 1 1])...
                  -(t0*t2+2*t5-t1-t3)*g...
                  +(2*t0*t2+t5-t1-t3)*gp2...
                  -(t0*t2-t1)*gp3);
end
xci = [xcihat(3,2);xcihat(1,3);xcihat(2,1);xcihat(1,4);xcihat(2,4);xcihat(3,4)];

% eof