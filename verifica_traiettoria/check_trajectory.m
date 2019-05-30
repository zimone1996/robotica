close all
clear all
clc




load('Traiettoria.mat');


a1=0.1050;
a2=0.10;
a3=0.11;
a4=0.11;

XY = [Pian(:,1),Pian(:,2),Pian(:,5)];
tf=1;
dt=tf/size(XY,1);
T=0:dt:tf-dt;

XYdot=[Pian(:,3:4),Pian(:,6)];
q4 = deg2rad(-60);
Q0=analitycal_IK_4DoF(XY(1,1:2),XY(1,3),a1,a2,a3,a4,q4)';
Q=Q0;


XY_IK1=[];
XY_IK2=[];
XY_IK3=[];
XY_IK4=[];
XY_err=[];
joints=[];
for i=1:size(XY,1)
    
    % Inversione cinematica
    Q_dot=inv_man_rid(Q,XY(i,1:3)',XYdot(i,1:3)'); %velocità angolare dei giunti q punto
    Q=Q+Q_dot*dt; %metodo di Eulero per calcolo variabili di giunto
    
    joints=[joints;Q'];
    [xy_ik1, xy_ik2, xy_ik3, xy_ik4]=kin_man_rid(Q); %faccio cinematica diretta per calcolo posa dei giunti
    
    XY_IK1=[XY_IK1; xy_ik1'];
    XY_IK2=[XY_IK2; xy_ik2'];
    XY_IK3=[XY_IK3; xy_ik3'];
    XY_IK4=[XY_IK4; xy_ik4'];
    
    
    XY_err=[XY_err; xy_ik4(1:3)'-XY(i,1:3)];
end


figure(1) %plotta variabili di giunto
subplot(4,1,1)
plot(T,joints(:,1),'-b','Linewidth',4)


subplot(4,1,2)
plot(T,joints(:,2),'-b','Linewidth',4)


subplot(4,1,3)
plot(T,joints(:,3),'-b','Linewidth',4)


subplot(4,1,4)
plot(T,joints(:,4),'-b','Linewidth',4)



figure(2) %plotta errore su organo terminale
subplot(3,1,1)
plot(T,XY_err(:,1),'-b','Linewidth',4)


subplot(3,1,2)
plot(T,XY_err(:,2),'-b','Linewidth',4)


subplot(3,1,3)
plot(T,XY_err(:,3),'-b','Linewidth',4)




MAKE_VIDEO = 1;
if(MAKE_VIDEO)
    motion = VideoWriter(['mov_2D_',datestr(now,30),'.avi']);
    open(motion);
end


figure(5)
for i=1:200:size(XY_IK1,1) %fino a lunghezza vettore di uscita inversione cinematica
    axis equal
    plot(XY(:,1),XY(:,2),'-k','Linewidth',4)
    hold on
    plot([0 XY_IK1(i,1)],[0 XY_IK1(i,2)],'-r','Linewidth',4)
    plot([XY_IK1(i,1) XY_IK2(i,1)],[XY_IK1(i,2) XY_IK2(i,2)],'-b','Linewidth',4)
    plot([XY_IK2(i,1) XY_IK3(i,1)],[XY_IK2(i,2) XY_IK3(i,2)],'-g','Linewidth',4)
    plot([XY_IK3(i,1) XY_IK4(i,1)],[XY_IK3(i,2) XY_IK4(i,2)],'-m','Linewidth',4)
    ylim([-0.1 0.4]);
    grid on
    
    if(MAKE_VIDEO)
        F = getframe(gcf); %prende il frame dal plot in quell'istane e lo aggiunge al video che sto creando
        writeVideo(motion,F); %aggiungo frame
    end
    pause(0.01)
    hold off %cancello quelloche ci stava prima
    
end



if(MAKE_VIDEO)
    close(motion);
end


