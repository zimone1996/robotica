
function XY=direct_kinematics_4DoF(Qdes,a1,a2,a3,a4)
q1 = Qdes(1);
q2 = Qdes(2);
q3 = Qdes(3);
q4 = Qdes(4);
%posa organo terminale
XY=[a1*cos(q1)+a2*cos(q1+q2)+a3*cos(q1+q2+q3)+a4*cos(q1+q2+q3+q4);
    a1*sin(q1)+a2*sin(q1+q2)+a3*sin(q1+q2+q3)+a4*sin(q1+q2+q3+q4);
    q1+q2+q3+q4];
end