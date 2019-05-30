
function [XY1, XY2, XY3, XY4]=kin_man_rid(Q)
r1=0.105;
r2=0.10;
r3=0.11;
r4=0.11;


q1=Q(1);
q2=Q(2);
q3=Q(3);
q4=Q(4);


XY1=[r1*cos(q1);
    r1*sin(q1);
    q1];

XY2=[r1*cos(q1)+r2*cos(q1+q2);
    r1*sin(q1)+r2*sin(q1+q2);
    q1+q2];

XY3=[r1*cos(q1)+r2*cos(q1+q2)+r3*cos(q1+q2+q3);
    r1*sin(q1)+r2*sin(q1+q2)+r3*sin(q1+q2+q3);
    q1+q2+q3];

XY4=[r1*cos(q1)+r2*cos(q1+q2)+r3*cos(q1+q2+q3)+r4*cos(q1+q2+q3+q4);
    r1*sin(q1)+r2*sin(q1+q2)+r3*sin(q1+q2+q3)+r4*sin(q1+q2+q3+q4);
    q1+q2+q3+q4];


end