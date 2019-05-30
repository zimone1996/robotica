%% Definizione dei parametri
clear all
clc
close all

syms q1 q2 q3 q4 dot_q1 dot_q2 dot_q3 dot_q4 'real' %variabili simboliche del modello

%% PARAMETRI DINAMICI DEL ROBOT - valori numerici
%Lunghezze link [m]
l1=0.1050;
l2=0.10;
l3=0.11;
l4=0.11;

%Sezione link [m]
% Si assume che i link siano parallelepipedi
a1=0.040; %larghezza piano xy
b1=0.0235; %spessore lungo z
%valido per 2 3 e 4 link
a=0.015;
b=0.0235;

%Rotore motori
%Si assume che il rotore del motore sia un cilindro 
%di altezza h e raggio r
%motore grande
H=0.025;
R=0.0116;

%motore piccolo
h=0.012;
r=0.008;

%Masse link [kg]
ml_1=0.017; %sommata con c2=0.011
ml_2=0.020; 
ml_3=0.022;
ml_4=0.024;

%Masse statori motori [kg] 90% del totale
%perchè sommo lo statore al primo link?
ms_1=0.9*0.081;
ms_2=0.9*0.081;
ms_3=0.9*0.038;
ms_4=0.9*0.038;

%Masse totali Link [kg]
M_1=ml_1+ms_2;
M_2=ml_2+ms_3;
M_3=ml_3+ms_4;
M_4=ml_4;


%Masse rotori [kg] 10% del totale
M_r1=0.1*0.081;
M_r2=0.1*0.081;
M_r3=0.1*0.038;
M_r4=0.1*0.038;

%Coefficienti di riduzione motori 
kr1=3.3;
kr2=2;
kr3=1.5;
kr4=1.2;
  

%% Centri di massa dei link 
%al centro dei link
COM_1=[-l1/2; 0; 0];     %Terna1 %a metà del link come parallelepipedo
COM_2=[-l2*0.55; 0; 0];     %Terna2
COM_3=[-l3*0.56; 0; 0];     %Terna3
COM_4=[-l4/2; 0; 0];     %Terna4

%al centro delle terne
%Centri di massa dei Motori/Rotori rispetto alla terna precedente
COM_r1=[0; 0; 0];        %Terna 0
COM_r2=[0; 0; 0];        %Terna 1
COM_r3=[0; 0; 0];        %Terna 2
COM_r4=[0; 0; 0];        %Terna 3


%% TENSORI INERZIA LINK [kgm^2]
%Link 1 rispetto cm %le terne coincidono con la terna centrale di
%inerzia ---> diagonali

I_1=[1/12*M_1*(a1^2+b1^2)          0                   0;
            0           1/12*M_1*(b1^2+l1^2)          0;
            0                    0          1/12*M_1*(a1^2+l1^2)];

%Link 2 rispetto Terna 2
I_2=[1/12*M_2*(a^2+b^2)          0                   0;
            0           1/12*M_2*(b^2+l2^2)          0;
            0                    0          1/12*M_2*(a^2+l2^2)];

%Link 3 Terna 3
I_3=[1/12*M_3*(a^2+b^2)          0                   0;
            0           1/12*M_3*(b^2+l3^2)          0;
            0                    0          1/12*M_3*(a^2+l3^2)];
        
        
 %Link 4 Terna 4
I_4=[1/12*M_4*(a^2+b^2)          0                   0;
            0           1/12*M_4*(b^2+l4^2)          0;
            0                    0          1/12*M_4*(a^2+l4^2)];
 
          
%% Tensori di inerzia dei Rotori [kgm^2]
% %Motore1 rispetto Terna 0
I_m1_b=[1/12*M_r1*(H^2+3*R^2)         0                 0;
                0             1/12*M_r1*(H^2+3*R^2)     0;
                0                     0             1/2*M_r1*R^2];
%Motore2 rispetto Terna 1
I_m2_b=[1/12*M_r2*(H^2+3*R^2)         0                 0;
                0             1/12*M_r2*(H^2+3*R^2)     0;
                0                     0             1/2*M_r2*R^2];
%Motore3 rispetto Terna 2
I_m3_b=[1/12*M_r3*(h^2+3*r^2)         0                 0;
                0             1/12*M_r3*(h^2+3*r^2)     0;
                0                     0             1/2*M_r3*r^2];
 %Motore4 rispetto Terna 3
I_m4_b=[1/12*M_r4*(h^2+3*r^2)         0                 0;
                0             1/12*M_r4*(h^2+3*r^2)     0;
                0                     0             1/2*M_r4*r^2];

%% Vettore g
g0=[0 0 -9.81]'; %robot ruotato che lavora contro gravità lungo asse y; in realtà noi avremo 
%gravità ortogonale al piano di lavoro ---> non produrrà alcuna coppia 

%% RISOLUZIONE SIMBOLICA CINEMATICA
Q=[q1; q2; q3;q4];
Q_dot=[dot_q1; dot_q2; dot_q3;dot_q4];

%calcolo matrici di trasformazione omogenea dei singoli giunti
T_01=calcolo_DH(0,l1,0,q1);
T_12=calcolo_DH(0,l2,0,q2); %rispetto la prima terna
T_23=calcolo_DH(0,l3,0,q3); %rispetto la seconda terna
T_34=calcolo_DH(0,l4,0,q4); %rispetto la terza terna
    
%riferisco alla terna zero
T_02=T_01*T_12; 
T_03=T_02*T_23; %cinematica diretta
T_04=T_03*T_34; %cinematica diretta


%Orientamento asse z delle terne di riferimento utili per jacobiano
%geometrico
z_0=[0;0;1];
z_1=T_01(1:3,3); %prendo asse z dalle matrici di trasformazioni omogenee terna colonna matrice di rotazione 
z_2=T_02(1:3,3);
z_3=T_03(1:3,3);
z_4=T_04(1:3,3);

%Posizione dell'Origine delle Terne rispetto alla terna base
p_0=[0;0;0]; %prendo posizione dalle prime tre componenti della quarta colonna della matrice di trasformazione
p_1=T_01(1:3,4);
p_2=T_02(1:3,4);
p_3=T_03(1:3,4);
p_e=T_04(1:3,4);

%% COMPONENTI DELLA LAGRANGIANA
%% Link 1
% Si esprime la posizione del baricentro del Link1 in terna 0 (base)
P1_=T_01*[COM_1;1]; %riporto in terna base 
P1=P1_(1:3); %prendo solo le prime 3 componenti
%Tensore di inerzia espresso in terna base
I_01=T_01(1:3,1:3)*I_1*T_01(1:3,1:3)';  %uso matrici di rotazione

%Jacobiano geometrico del link 1 : sarà una matrice 6x4
JP1_l1=cross(z_0,P1);
JP_l1=[JP1_l1 zeros(3,3)]; %aggiungo zeri perchè altri due giunti 

JO1_l1=z_0;
JO_l1=[JO1_l1 zeros(3,3)];

%% Link 2 faccio lo stesso rispetto quanto fatto per il link 1
P2_=T_02*[COM_2;1];
P2=P2_(1:3);
I_02=T_02(1:3,1:3)*(I_2)*T_02(1:3,1:3)';

%Jacobiano geometrico del link 2
JP1_l2=cross(z_0,P2);
JP2_l2=cross(z_1,P2-p_1);
JP_l2=[JP1_l2 JP2_l2 zeros(3,2)];

JO1_l2=z_0;
JO2_l2=z_1;
JO_l2=[JO1_l2 JO2_l2 zeros(3,2)];

%% Link 3 faccio lo stesso rispetto quanto fatto per il link 2
P3_=T_03*[COM_3;1];
P3=P3_(1:3);
I_03=T_03(1:3,1:3)*(I_3)*T_03(1:3,1:3)';

%Jacobiano geometrico del link 3
JP1_l3=cross(z_0,P3);
JP2_l3=cross(z_1,P3-p_1);
JP3_l3=cross(z_2,P3-p_2);
JP_l3=[JP1_l3 JP2_l3 JP3_l3 zeros(3,1)];

JO1_l3=z_0;
JO2_l3=z_1;
JO3_l3=z_2;
JO_l3=[JO1_l3 JO2_l3 JO3_l3 zeros(3,1)];

%% Link 4 faccio lo stesso rispetto quanto fatto per il link 3
P4_=T_04*[COM_4;1];
P4=P4_(1:3);
I_04=T_04(1:3,1:3)*(I_4)*T_04(1:3,1:3)';

%Jacobiano geometrico del link 4
JP1_l4=cross(z_0,P4);
JP2_l4=cross(z_1,P4-p_1);
JP3_l4=cross(z_2,P4-p_2);
JP4_l4=cross(z_3,P4-p_3);
JP_l4=[JP1_l4 JP2_l4 JP3_l4 JP4_l4];

JO1_l4=z_0;
JO2_l4=z_1;
JO3_l4=z_2;
JO4_l4=z_3;
JO_l4=[JO1_l4 JO2_l4 JO3_l4 JO4_l4];

%% Motori
% Motore 1. Esprimo la posizione del baricentro del motore in terna base e 
% riporto il tensore di inerzia in terna base
%Ovviamente il baricentro del primo rotore è al centro della terna zero 
%Consideriamo il motore 1 perchè il rotore non è solidale con la base
pm1=COM_r1;  %coordinate baricentro espresse in terna base
I_m1=I_m1_b; %tensore di inerzia è già espresso in terna base

JP_m1=zeros(3,4);

JO1_m1=kr1*z_0;
JO_m1=[JO1_m1 zeros(3,3)];

% Motore 2
pm2=T_01(1:3,1:3)*COM_r2;
I_m2=T_01(1:3,1:3)*I_m2_b*T_01(1:3,1:3)';

JP1_m2=cross(z_0,pm2-p_0);
JP_m2=[JP1_m2 zeros(3,3)];

JO2_m2=kr2*z_1;
JO_m2=[JO1_l2 JO2_m2 zeros(3,2)];

% Motore 3
pm3=T_02(1:3,1:3)*COM_r3;
I_m3=T_02(1:3,1:3)*I_m3_b*T_02(1:3,1:3)';

JP1_m3=cross(z_0,pm3-p_0);
JP2_m3=cross(z_1,pm3-p_1);
JP_m3=[JP1_m3 JP2_m3 zeros(3,2)];

JO3_m3=kr3*z_2;
JO_m3=[JO1_l3 JO2_l3 JO3_m3 zeros(3,1)];

% Motore 4
pm4=T_03(1:3,1:3)*COM_r4;
I_m4=T_03(1:3,1:3)*I_m4_b*T_03(1:3,1:3)';

JP1_m4=cross(z_0,pm4-p_0);
JP2_m4=cross(z_1,pm4-p_1);
JP3_m4=cross(z_2,pm4-p_2);
JP_m4=[JP1_m4 JP2_m4 JP3_m4 zeros(3,1)];

JO4_m4=kr4*z_3;
JO_m4=[JO1_l4 JO2_l4 JO3_l4 JO4_m4];

