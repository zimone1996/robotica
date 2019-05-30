function tau= PID_controller(Q, Qdot, Qi, Qdes, Qdotdes, Qides,K_,D_,I_)

K=[K_(1)   0       0        0;
   0      K_(2)    0        0;
   0       0     K_(3)      0;
   0       0       0     K_(4)];

D=[D_(1)   0       0        0;
   0      D_(2)    0        0;
   0       0     D_(3)      0;
   0       0       0     D_(4)];

I=[I_(1)   0       0       0;
   0      I_(2)    0       0;
   0       0     I_(3)     0;
   0       0       0     I_(4)];


tau=K*(Qdes-Q)+D*(Qdotdes-Qdot)+I*(Qides-Qi);


end