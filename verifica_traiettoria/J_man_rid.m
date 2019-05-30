
function J=J_man_rid(Q)
q1=Q(1);
q2=Q(2);
q3=Q(3);
q4=Q(4);


r1=0.1050;
r2=0.10;
r3=0.11;
r4=0.11;


J=[-r1*sin(q1)-r2*sin(q1+q2)-r3*sin(q1+q2+q3)-r4*sin(q1+q2+q3+q4) -r2*sin(q1+q2)-r3*sin(q1+q2+q3)-r4*sin(q1+q2+q3+q4) -r3*sin(q1+q2+q3)-r4*sin(q1+q2+q3+q4) -r4*sin(q1+q2+q3+q4);
    r1*cos(q1)+r2*cos(q1+q2)+r3*cos(q1+q2+q3)+r4*cos(q1+q2+q3+q4)  r2*cos(q1+q2)+r3*cos(q1+q2+q3)+r4*cos(q1+q2+q3+q4) r3*cos(q1+q2+q3)+r4*cos(q1+q2+q3+q4) +r4*cos(q1+q2+q3+q4);
   1 1 1 1];


end