function Qdes= traiettoria_iniziale(q,t,tf,ti)
    qi=[0;0;0;0];
    qf = q;
    qidot=[0;0;0;0];
    qfdot=[0;0;0;0];
    qidotdot=[0;0;0;0];
    qfdotdot=[0;0;0;0];
  


   A=[ti^5 ti^4 ti^3 ti^2 ti 1;
   tf^5 tf^4 tf^3 tf^2 tf 1;
   5*ti^4 4*ti^3 3*ti^2 2*ti 1 0;
   5*tf^4 4*tf^3 3*tf^2 2*tf 1 0;
   20*ti^3 12*ti^2 6*ti 2 0 0;
   20*tf^3 12*tf^2 6*tf 2 0 0];

B1=[qi(1);qf(1);qidot(1);qfdot(1);qidotdot(1);qfdotdot(1)];
B2=[qi(2);qf(2);qidot(2);qfdot(2);qidotdot(2);qfdotdot(2)];
B3=[qi(3);qf(3);qidot(3);qfdot(3);qidotdot(3);qfdotdot(3)];
B4=[qi(4);qf(4);qidot(4);qfdot(4);qidotdot(4);qfdotdot(4)];

a1=inv(A)*B1;
a2=inv(A)*B2;
a3=inv(A)*B3;
a4=inv(A)*B4;

q1 = a1(1)*t^5+a1(2)*t^4+a1(3)*t^3+a1(4)*t^2+a1(5)*t+a1(6);
q2 = a2(1)*t^5+a2(2)*t^4+a2(3)*t^3+a2(4)*t^2+a2(5)*t+a2(6);
q3 = a3(1)*t^5+a3(2)*t^4+a3(3)*t^3+a3(4)*t^2+a3(5)*t+a3(6);
q4 = a4(1)*t^5+a4(2)*t^4+a4(3)*t^3+a4(4)*t^2+a4(5)*t+a4(6);

Qdes=[q1;q2;q3;q4];

end