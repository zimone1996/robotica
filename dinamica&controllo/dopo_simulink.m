

Q_=out.simout1.Data;

    XY1=[];
    XY2=[];
    XY3=[];
    XY4=[];
    

for i =1:size(Q_)
    [xy1, xy2, xy3, xy4]=kin_man_rid(Q_(i,:));
    
    XY1=[XY1; xy1'];
    XY2=[XY2; xy2'];
    XY3=[XY3; xy3'];
    XY4=[XY4; xy4'];
    
end

MAKE_VIDEO = 1;
if(MAKE_VIDEO)
    motion = VideoWriter(['mov_2D_',datestr(now,30),'.avi']);
    open(motion);
end

figure
load('Traiettoria.mat');
for i=1:40:size(XY1,1)
    axis equal
%     plot([XYi(1) XYf(1)],[XYi(2) XYf(2)],'-k','Linewidth',4) 
%     hold on
%     plot([XYi_c1(1) XYf_c1(1)],[XYi_c1(2) XYf_c1(2)],'-m','Linewidth',4)  
%     hold on
%     plot([XYi_c2(1) XYf_c2(1)],[XYi_c2(2) XYf_c2(2)],'-b','Linewidth',4) 
    plot(Pian(:,1),Pian(:,2),'k','LineWidth',4)
    hold on
    plot([0 XY1(i,1)],[0 XY1(i,2)],'-r','Linewidth',4)    
    plot([XY1(i,1) XY2(i,1)],[XY1(i,2) XY2(i,2)],'-b','Linewidth',4)
    plot([XY2(i,1) XY3(i,1)],[XY2(i,2) XY3(i,2)],'-g','Linewidth',4)
    plot([XY3(i,1) XY4(i,1)],[XY3(i,2) XY4(i,2)],'-m','Linewidth',4)
    axis equal
    
    if(MAKE_VIDEO)
        F = getframe(gcf);
        writeVideo(motion,F);
    end
    %pause(0.5)
    hold off
    
end



if(MAKE_VIDEO)
    close(motion);
end

