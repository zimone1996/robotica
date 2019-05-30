clear all
close all
clc

%% Variables initialization
a1=0.105;
a2=0.10;
a3=0.11;
a4=0.11;

XYi=[0.25 0.2 0];
XYf=[0.25 0.3 0.2618];

XYi_c1=[0.25 0.3 0.2618];
XYf_c1=[0.25 0.25 0.5236];


XYi_c2=[0.25 0.25 0.5236];
XYf_c2=[0.25 0.2 0.7854];

Qides=[0;0;0;0];
q4 = deg2rad(0);
Qdes=analitycal_IK_4DoF(XYi(1:2),XYi(3),a1,a2,a3,a4,q4)';
Qdotdes=[0;0;0;0];
Q2dotdes=[0;0;0;0];

Qi=[0;0;0;0];
Q=Qdes;
Qdot=[0;0;0;0];
Q2dot=[0;0;0;0];

tau=[0;0;0;0];
Q2dot_=[];
Qdot_=[];
Q_=[];

XY1=[];
XY2=[];
XY3=[];
XY4=[];

Qdotdes_=[];
Qdes_=[];


XY_err_IK=[];

 q_des=[];

tf=12;

T(1)=0.0;
time=0.0;
i=2;
start_time=tic;
ix=[];
iy=[];
%% Control loop
while(time<tf)
    time=toc(start_time);
    T(i)=time;
    dt=T(i)-T(i-1);
    
    if time < 4
    %% Online Trajectory planning nello spazio operativo...mi servirà inversione cinematica
    IN_XY(1:2)=XYi(1:2);
    IN_XY(3:4)=XYf(1:2);
    IN_XY(5)=0;
    IN_XY(6)=4;
    IN_XY(7)=time;
    
   
    [XY_, XYdot_, phi_, phidot_]=segmento(IN_XY);
    end
    
     if  time > 4 && time < 8
    %% Online Trajectory planning nello spazio operativo...mi servirà inversione cinematica
    IN_XY(1:2)=XYi_c1(1:2);
    IN_XY(3:4)=XYf_c1(1:2);
    IN_XY(5)=4;
    IN_XY(6)=8;
    IN_XY(7)=time;
    
    [XY_, XYdot_, phi_, phidot_]=circonferenza1(IN_XY);
     end
    
     
      if time > 8 && time < 12
    %% Online Trajectory planning nello spazio operativo...mi servirà inversione cinematica
    IN_XY(1:2)=XYi_c2(1:2);
    IN_XY(3:4)=XYf_c2(1:2);
    IN_XY(5)=8;
    IN_XY(6)=12;
    IN_XY(7)=time;
    
    [XY_, XYdot_, phi_, phidot_]=circonferenza2(IN_XY);
      end
    
      
      
    XY = [XY_';phi_];
    ix=[ix;XY_(1)];
    iy=[iy;XY_(2)];
    XYdot=[XYdot_';phidot_];
    %% Online inverse kinematics
    %gestire ridondanza
    Q_dotdes=inv_man_rid(Qdes,XY,XYdot);
    Qdes=Qdes+Q_dotdes*dt;
    q_des=[q_des;Qdes'];
    

    %Q_dotdes=inv_man_rid(Q,XY,XYdot,a1,a2,a3);
    %Qdes=Q+Q_dotdes*dt;
    Qides=Qides+Qdes*dt;
    
    XY_IK=direct_kinematics_4DoF(Qdes,a1,a2,a3,a4);
    XY_err_IK=[XY_err_IK; XY_IK(1:3)'-XY(1:3)'];%errore inversione cinematica
    %% PID controller


K=[1 0 0 0]; 
D=[0 0 0 0];
I=[0 0 0 0];


% 
% K=[900 0 0 0];
% D=[50 0 0 0   ];
% I=[0 0 0 0];
% 

%      K %trovo T=0.26 e Kp= 160
%      I=[0 0 0 0];
%      D=[0 0 0 0];%0.1*0.6*160/8
%     
    tau= PID_controller(Q, Qdot, Qi, Qdes, Qdotdes, Qides, K, D, I);
    
    %% Robot dynamic model
    Q2dot=dynamic_model_4dof(tau,Q, Qdot);
    Qdot=Qdot+Q2dot*dt;
    Q=Q+Qdot*dt;
    Qi=Qi+Q*dt;
    
    
    
    %% Real variable saving
    Q2dot_=[Q2dot_;Q2dot'];
    Qdot_=[Qdot_;Qdot'];
    Q_=[Q_;Q'];
    
    [xy1, xy2, xy3, xy4]=kin_man_rid(Q);
    
    XY1=[XY1; xy1'];
    XY2=[XY2; xy2'];
    XY3=[XY3; xy3'];
    XY4=[XY4; xy4'];
    
    %% Desired variable saving
    
    Qdotdes_=[Qdotdes_;Qdotdes'];
    Qdes_=[Qdes_;Qdes'];
    
    i=i+1;
end

%% Downsampling
sample_number=200;

Q_=Q_(1:end/sample_number:end,:);
Qdes_=Qdes_(1:end/sample_number:end,:);
XY_err_IK=XY_err_IK(1:end/sample_number:end,:);
T=T(1:(end-1)/sample_number:end-1);

%% Plot
figure(1)
% subplot(4,1,1)
plot(T,Q_(:,1),'-b','Linewidth',1)
hold on
plot(T,Qdes_(:,1),'-r','Linewidth',1)

% subplot(4,1,2)
figure
plot(T,Q_(:,2),'-b','Linewidth',1)
hold on
plot(T,Qdes_(:,2),'-r','Linewidth',1)

figure
plot(T,Q_(:,3),'-b','Linewidth',1)
hold on
plot(T,Qdes_(:,3),'-r','Linewidth',1)

figure
plot(T,Q_(:,4),'-b','Linewidth',1)
hold on
plot(T,Qdes_(:,4),'-r','Linewidth',1)


figure
plot(T,rad2deg(Q_(:,1)-Qdes_(:,1)),'-b','Linewidth',1)

figure
plot(T,rad2deg(Q_(:,2)-Qdes_(:,2)),'-b','Linewidth',1)

% figure
% plot(T,rad2deg(Q_(:,3)-Qdes_(:,3)),'-b','Linewidth',1)
% 
% figure
% plot(T,rad2deg(Q_(:,4)-Qdes_(:,4)),'-b','Linewidth',1)
% subplot(4,1,3)
% plot(T,Q_(:,3),'-b','Linewidth',4)
% hold on
% plot(T,Qdes_(:,3),'-r','Linewidth',4)
% 
% subplot(4,1,4)
% plot(T,Q_(:,4),'-b','Linewidth',4)
% hold on
% plot(T,Qdes_(:,4),'-r','Linewidth',4)


figure(2)
subplot(3,1,1)
plot(T,XY_err_IK(:,1),'-b','Linewidth',4)

subplot(3,1,2)
plot(T,XY_err_IK(:,2),'-b','Linewidth',4)

subplot(3,1,3)
plot(T,XY_err_IK(:,3),'-b','Linewidth',4)


% 
% 
% MAKE_VIDEO = 1;
% if(MAKE_VIDEO)
%     motion = VideoWriter(['mov_2D_',datestr(now,30),'.avi']);
%     open(motion);
% end
% 
% 
% figure
% load('Traiettoria.mat');
% for i=1:40:size(XY1,1)
%     axis equal
% %     plot([XYi(1) XYf(1)],[XYi(2) XYf(2)],'-k','Linewidth',4) 
% %     hold on
% %     plot([XYi_c1(1) XYf_c1(1)],[XYi_c1(2) XYf_c1(2)],'-m','Linewidth',4)  
% %     hold on
% %     plot([XYi_c2(1) XYf_c2(1)],[XYi_c2(2) XYf_c2(2)],'-b','Linewidth',4) 
%     plot(Pian(:,1),Pian(:,2),'k','LineWidth',4)
%     hold on
%     plot([0 XY1(i,1)],[0 XY1(i,2)],'-r','Linewidth',4)    
%     plot([XY1(i,1) XY2(i,1)],[XY1(i,2) XY2(i,2)],'-b','Linewidth',4)
%     plot([XY2(i,1) XY3(i,1)],[XY2(i,2) XY3(i,2)],'-g','Linewidth',4)
%     plot([XY3(i,1) XY4(i,1)],[XY3(i,2) XY4(i,2)],'-m','Linewidth',4)
%     axis equal
%     
%     if(MAKE_VIDEO)
%         F = getframe(gcf);
%         writeVideo(motion,F);
%     end
%     %pause(0.5)
%     hold off
%     
% end
% 
% 
% 
% if(MAKE_VIDEO)
%     close(motion);
% end
% 
