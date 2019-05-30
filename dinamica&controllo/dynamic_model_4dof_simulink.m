function Q2dot =Copy_of_dynamic_model_4dof(u)
%resittuisce in uscita l'accelerazione del modello
tau = u(1:4);
q1=u(9);
q2=u(10);
q3=u(11);
q4=u(12);


dot_q1=u(5);
dot_q2=u(6);
dot_q3=u(7);
dot_q4=u(8);
                                                
 
B = [ (693*cos(q2 + q3 + q4))/2500000 + (1407021*cos(q2 + q3))/1250000000 + (33*cos(q3 + q4))/125000 + (219639*cos(q2))/100000000 + (67001*cos(q3))/62500000 + (363*cos(q4))/1250000 + 15522076941012894553906823163/4611686018427387904000000000000, (693*cos(q2 + q3 + q4))/5000000 + (1407021*cos(q2 + q3))/2500000000 + (33*cos(q3 + q4))/125000 + (219639*cos(q2))/200000000 + (67001*cos(q3))/62500000 + (363*cos(q4))/1250000 + 7084910208573392023919885663/4611686018427387904000000000000, (693*cos(q2 + q3 + q4))/5000000 + (1407021*cos(q2 + q3))/2500000000 + (33*cos(q3 + q4))/250000 + (67001*cos(q3))/125000000 + (363*cos(q4))/1250000 + 10649821038632019207492447027/18446744073709551616000000000000, (693*cos(q2 + q3 + q4))/5000000 + (33*cos(q3 + q4))/250000 + (363*cos(q4))/2500000 + 143731008805079166613853/1475739525896764129280000000;
  (693*cos(q2 + q3 + q4))/5000000 + (1407021*cos(q2 + q3))/2500000000 + (33*cos(q3 + q4))/125000 + (219639*cos(q2))/200000000 + (67001*cos(q3))/62500000 + (363*cos(q4))/1250000 + 7084910208573392023919885663/4611686018427387904000000000000,                                                                                                    (33*cos(q3 + q4))/125000 + (67001*cos(q3))/62500000 + (363*cos(q4))/1250000 + 7476349822669603530224170663/4611686018427387904000000000000,                                              (33*cos(q3 + q4))/250000 + (399*cos(q2))/10000000 + (67001*cos(q3))/125000000 + (363*cos(q4))/1250000 + 11422647381600080872444767027/18446744073709551616000000000000,                                   (33*cos(q3 + q4))/250000 + (363*cos(q4))/2500000 + 143731008805079166613853/1475739525896764129280000000;
                           (693*cos(q2 + q3 + q4))/5000000 + (1407021*cos(q2 + q3))/2500000000 + (33*cos(q3 + q4))/250000 + (67001*cos(q3))/125000000 + (363*cos(q4))/1250000 + 10649821038632019207492447027/18446744073709551616000000000000,                                                                        (33*cos(q3 + q4))/250000 + (399*cos(q2))/10000000 + (67001*cos(q3))/125000000 + (363*cos(q4))/1250000 + 11422647381600080872444767027/18446744073709551616000000000000,                                                                                                      (399*cos(q2))/5000000 + (363*cos(q4))/1250000 + 24250611998921132290019987179/36893488147419103232000000000000,                                                              (363*cos(q4))/2500000 + 143731008805079166613853/1475739525896764129280000000;
                                                                                                     (693*cos(q2 + q3 + q4))/5000000 + (33*cos(q3 + q4))/250000 + (363*cos(q4))/2500000 + 143731008805079166613853/1475739525896764129280000000,                                                                                                                                      (33*cos(q3 + q4))/250000 + (363*cos(q4))/2500000 + 143731008805079166613853/1475739525896764129280000000,                                                                                                                                       (363*cos(q4))/2500000 + 143731008805079166613853/1475739525896764129280000000,                                                                                        71887038393701468889739/737869762948382064640000000];   

C = [ - dot_q4*((693*sin(q2 + q3 + q4))/5000000 + (33*sin(q3 + q4))/250000 + (363*sin(q4))/2500000) - dot_q2*((693*sin(q2 + q3 + q4))/5000000 + (1407021*sin(q2 + q3))/2500000000 + (219639*sin(q2))/200000000) - dot_q3*((693*sin(q2 + q3 + q4))/5000000 + (1407021*sin(q2 + q3))/2500000000 + (33*sin(q3 + q4))/250000 + (67001*sin(q3))/125000000), - dot_q4*((693*sin(q2 + q3 + q4))/5000000 + (33*sin(q3 + q4))/250000 + (363*sin(q4))/2500000) - dot_q3*((693*sin(q2 + q3 + q4))/5000000 + (1407021*sin(q2 + q3))/2500000000 + (33*sin(q3 + q4))/250000 + (67001*sin(q3))/125000000) - (21*(dot_q1 + dot_q2)*(33000*sin(q2 + q3 + q4) + 134002*sin(q2 + q3) + 261475*sin(q2)))/5000000000, - dot_q4*((693*sin(q2 + q3 + q4))/5000000 + (33*sin(q3 + q4))/250000 + (363*sin(q4))/2500000) - (11*(dot_q1 + dot_q2 + dot_q3)*(31500*sin(q2 + q3 + q4) + 127911*sin(q2 + q3) + 30000*sin(q3 + q4) + 121820*sin(q3)))/2500000000, -(33*(21*sin(q2 + q3 + q4) + 20*sin(q3 + q4) + 22*sin(q4))*(dot_q1 + dot_q2 + dot_q3 + dot_q4))/5000000;
                                                                                                           dot_q1*((693*sin(q2 + q3 + q4))/5000000 + (1407021*sin(q2 + q3))/2500000000 + (219639*sin(q2))/200000000) - dot_q3*((33*sin(q3 + q4))/250000 + (67001*sin(q3))/125000000) - dot_q4*((33*sin(q3 + q4))/250000 + (363*sin(q4))/2500000),                                                                                                                                                                                                              - dot_q4*((33*sin(q3 + q4))/250000 + (363*sin(q4))/2500000) - dot_q3*((33*sin(q3 + q4))/250000 + (67001*sin(q3))/125000000),       - (11*(dot_q1 + dot_q2)*(1500*sin(q3 + q4) + 6091*sin(q3)))/125000000 - dot_q4*((33*sin(q3 + q4))/250000 + (363*sin(q4))/2500000) - dot_q3*((33*sin(q3 + q4))/250000 - (399*sin(q2))/10000000 + (67001*sin(q3))/125000000),                        -(33*(10*sin(q3 + q4) + 11*sin(q4))*(dot_q1 + dot_q2 + dot_q3 + dot_q4))/2500000;
                                                                                                             dot_q2*((33*sin(q3 + q4))/250000 + (67001*sin(q3))/125000000) - (363*dot_q4*sin(q4))/2500000 + dot_q1*((693*sin(q2 + q3 + q4))/5000000 + (1407021*sin(q2 + q3))/2500000000 + (33*sin(q3 + q4))/250000 + (67001*sin(q3))/125000000),                                                                                                                    dot_q1*((33*sin(q3 + q4))/250000 + (67001*sin(q3))/125000000) - (399*dot_q3*sin(q2))/10000000 - (363*dot_q4*sin(q4))/2500000 + dot_q2*((33*sin(q3 + q4))/250000 - (399*sin(q2))/10000000 + (67001*sin(q3))/125000000),                                                                                                                                                                   - (399*dot_q2*sin(q2))/10000000 - (363*dot_q4*sin(q4))/2500000,                                              -(363*sin(q4)*(dot_q1 + dot_q2 + dot_q3 + dot_q4))/2500000;
                                                                                                                                                          dot_q1*((693*sin(q2 + q3 + q4))/5000000 + (33*sin(q3 + q4))/250000 + (363*sin(q4))/2500000) + dot_q2*((33*sin(q3 + q4))/250000 + (363*sin(q4))/2500000) + (363*dot_q3*sin(q4))/2500000,                                                                                                                                                                                                                                             (33*(dot_q1 + dot_q2)*(10*sin(q3 + q4) + 11*sin(q4)))/2500000 + (363*dot_q3*sin(q4))/2500000,                                                                                                                                                                                 (363*sin(q4)*(dot_q1 + dot_q2 + dot_q3))/2500000,                                                                                                       0];
                                                                                                 
 G =[0;
    0;
                                                                                                          0;
                                                                                                                                                                      0];
                                                                                                                                                      
Fv=[1,   0.0,   0.0, 0;
    0.0,   1,   0.0, 0;
    0.0,   0.0,   1, 0;
    0, 0, 0, 1]*1e-1;
           
Qdot = [dot_q1;dot_q2;dot_q3;dot_q4];
%dinamica diretta          
Q2dot=inv(B)*(tau-C*Qdot-Fv*sign(Qdot)+G);

end