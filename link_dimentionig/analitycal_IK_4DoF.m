function Q=analitycal_IK_4DoF(p_,theta_,a1,a2,a3,a4,q4)
%Applicazione del problema cinematico inverso (Noti p e theta->si vuole ricavare Q)
p(1) = p_(1) - a4*cos(theta_); %posizione del terzo giunto...x
p(2) = p_(2) - a4*sin(theta_); %posizione del terzo giunto...y

theta = theta_-q4;

pw_x=p(1)-a3*cos(theta); %posizione secondo giunto...x
pw_y=p(2)-a3*sin(theta); %posizione secondo giunto...y


 
c2=(pw_x^2+pw_y^2-a1^2-a2^2)/(2*a1*a2);
s2=+sqrt(1-c2^2);

s1=((a1+a2*c2)*pw_y-a2*s2*pw_x)/(pw_x^2+pw_y^2);
c1=((a1+a2*c2)*pw_x+a2*s2*pw_y)/(pw_x^2+pw_y^2);

Q=[];
if (isreal(s1) && isreal(s2) && isreal(c1) && isreal(c2)) %verifico se valori sono reali
    q1=atan2(s1,c1); %calcolo q1
    q2=atan2(s2,c2); %calcolo q2
    q3=theta-q1-q2; %calcolo q3
    XY = direct_kinematics_4DoF([q1,q2,q3,q4],a1,a2,a3,a4);%viene applicata la cinematica diretta per verificare 
   % la correttezza delle variabili di giunto
    
    if(abs(XY-[p_(1);p_(2);theta_])<1e-3) %scostamento tra posa desiderata e posa calcolata con quelle variabili di giunto
        Q=[q1 q2 q3 q4];  %ritorno variabili d giunto
    end
    
end
end