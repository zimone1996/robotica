syms q1 q2 q3 q4 r1 r2 r3 r4 'real';
J=[-r1*sin(q1)-r2*sin(q1+q2)-r3*sin(q1+q2+q3)-r4*sin(q1+q2+q3+q4) -r2*sin(q1+q2)-r3*sin(q1+q2+q3)-r4*sin(q1+q2+q3+q4) -r3*sin(q1+q2+q3)-r4*sin(q1+q2+q3+q4) -r4*sin(q1+q2+q3+q4);
    r1*cos(q1)+r2*cos(q1+q2)+r3*cos(q1+q2+q3)+r4*cos(q1+q2+q3+q4)  r2*cos(q1+q2)+r3*cos(q1+q2+q3)+r4*cos(q1+q2+q3+q4) r3*cos(q1+q2+q3)+r4*cos(q1+q2+q3+q4) +r4*cos(q1+q2+q3+q4);
   1 1 1 1];

W = sqrt(det(J*J'));
dW = gradient(W,[q1,q2,q3,q4]);


