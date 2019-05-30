% Definisco le variabili
% Durata del moto
stop_time=12; %secondi
start_time1 = 0;
stop_time1 = 4;
start_time2 = 4;
stop_time2 = 8;
start_time3 = 8;
stop_time3 = 12;
%passo di integrazione numerica
dt=0.001;
% pi=IN(1:2);   posizione iniziale 
% pf=IN(3:4);   posizione finale 
% ti=IN(5);     tempo iniziale
% tf=IN(6);     tempo finale


% Condizioni iniziali così codificate C=[Pos_iniziale_x Pos_iniziale_y Pos_finale_x Pos_finale_y tempo_iniziale tempo_finale]
Cost=[0.25 0.2 0.25 0.3 start_time1 stop_time1 ];% metri_ sec
Cost1=[0.25 0.3 0.25 0.25 start_time2 stop_time2 ];
Cost2=[0.25 0.25 0.25 0.2 start_time3 stop_time3 ];

% matrice finale che ha per colonne Pian = [X,Y,v_X,v_Y,phi,omega]
% Pian = [[XYD_linea.signals.values(1:1/dt*stop_time1,1);XYD_circonferenza1.signals.values(1/dt*start_time2+1:1/dt*stop_time2,1);XYD_circonferenza2.signals.values(1/dt*start_time3+1:1/dt*stop_time3,1)],...
%     [XYD_linea.signals.values(1:1/dt*stop_time1,2);XYD_circonferenza1.signals.values(1/dt*start_time2+1:1/dt*stop_time2,2);XYD_circonferenza2.signals.values(1/dt*start_time3+1:1/dt*stop_time3,2)],...
%     [XYD_linea.signals.values(1:1/dt*stop_time1,3);XYD_circonferenza1.signals.values(1/dt*start_time2+1:1/dt*stop_time2,3);XYD_circonferenza2.signals.values(1/dt*start_time3+1:1/dt*stop_time3,3)],...
%     [XYD_linea.signals.values(1:1/dt*stop_time1,4);XYD_circonferenza1.signals.values(1/dt*start_time2+1:1/dt*stop_time2,4);XYD_circonferenza2.signals.values(1/dt*start_time3+1:1/dt*stop_time3,4)],...
%     [XYD_linea.signals.values(1:1/dt*stop_time1,5);XYD_circonferenza1.signals.values(1/dt*start_time2+1:1/dt*stop_time2,5);XYD_circonferenza2.signals.values(1/dt*start_time3+1:1/dt*stop_time3,5)],...
%     [XYD_linea.signals.values(1:1/dt*stop_time1,6);XYD_circonferenza1.signals.values(1/dt*start_time2+1:1/dt*stop_time2,6);XYD_circonferenza2.signals.values(1/dt*start_time3+1:1/dt*stop_time3,6)]];
%plot(Pian(:,1),Pian(:,2))
xlabel('asse x [m]');
ylabel('asse y [m]');
title('Traiettoria desiderata');
% axis equal
grid on