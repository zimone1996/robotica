function [XYd,XYddot,phi,phid] = circonferenza2(u)
r=(u(2)-u(4))/2;

    ti=u(5);
    tf=u(6);
    t=u(7);
    A=[ti^5 ti^4 ti^3 ti^2 ti 1;
   tf^5 tf^4 tf^3 tf^2 tf 1;
   
   5*ti^4 4*ti^3 3*ti^2 2*ti 1 0;
   5*tf^4 4*tf^3 3*tf^2 2*tf 1 0;
   20*ti^3 12*ti^2 6*ti 2 0 0;
   20*tf^3 12*tf^2 6*tf 2 0 0];
   B=[pi*r/2;-pi*r/2;0;0;0;0];
   a=inv(A)*B;
   s=a(1)*t^5+a(2)*t^4+a(3)*t^3+a(4)*t^2+a(5)*t+a(6);
   sdot=5*a(1)*t^4+4*a(2)*t^3+3*a(3)*t^2+2*a(4)*t+a(5);
    
   %coordinate delle circonferenze al centro dei due segmenti
    
    y_C =r + u(4); 
    x_C = u(1);
    C1 = [x_C;y_C];

  
   
   P1 = [r*cos(s/r);
       r*sin(s/r)
       ];
   
   P1dot = [-sdot*sin(s/r);
       sdot*cos(s/r)
       ];
   
   P = P1+C1;
   
   Pdot = P1dot;
   
   
   phi_i = pi/6;
   phi_f = pi/4;
   
   B1=[phi_i;phi_f;0;0;0;0];
   a1=inv(A)*B1;
   s1=a1(1)*t^5+a1(2)*t^4+a1(3)*t^3+a1(4)*t^2+a1(5)*t+a1(6);
   s1dot=5*a1(1)*t^4+4*a1(2)*t^3+3*a1(3)*t^2+2*a1(4)*t+a1(5);
   
   
   if t<ti %tempo antecedente ad istante di inizio
    XYd=u(1:2)'; %voglio posizione iniziale e quindi velocità nulle
    XYddot=[0;0]'; 
     phi = phi_i';
    phid = 0';
else
    if t<tf
        XYd=P'; %traiettoria rettilinea (segmento)...posizione
        XYddot=Pdot'; %... velocità
        phi = s1';
        phid = s1dot';
    else
        XYd=u(3:4)'; %se t maggiore del tempo finale devo essere in posizione finale
        XYddot=[0;0]';
        phi = phi_f';
        phid = 0';
    end
   end

% Prima del tempo iniziale mantiene posizione inizale e velocità nulla
% Dopo del tempo finale mantiene posizione finale e velocità nulla

