close all
%a = load('/home/jose/ORB-SDTAM/Examples/Monocular/LocalNeigh/errorframeL.txt');
sinit = 1118;

a = load('/home/jose/ORB-SDTAM/Examples/Monocular/strech/nonfixedcam/2+0/2/errorframe.txt');
s = load('/home/jose/ORB-SDTAM/Examples/Monocular/strech/nonfixedcam/2+0/2/status.txt');
s2 = load('/home/jose/ORB-SDTAM/Examples/Monocular/strech/nonfixedcam/2+1/2/status.txt');
s3 = load('/home/jose/ORB-SDTAM/Examples/Monocular/status.txt');
figure()
subplot(2,1,1);
plot([s(1:sinit,1);s(sinit+1:end,1)-146],0.3*s(:,2), 'LineWidth',1,'color','m');
subplot(2,1,2);
plot([s2(1:sinit,1);s2(sinit+1:end,1)-146],0.3*s2(:,2), 'LineWidth',1,'color','g');
%subplot(3,1,3);
%plot([s3(1:sinit,1);s3(sinit+1:end,1)-146],0.3*s3(:,2), 'LineWidth',1,'color','r');

%a2 = load('/home/jose/ORB-SDTAM/Examples/Monocular/errorframe2.txt');
%a3 = load('/home/jose/ORB-SDTAM/Examples/Monocular/errorframe3.txt');
init = 380;
b1=a(1:init,:);
b2=a(init+1:end,:);%[a2(380:end,:);a3(385:end,:)]
b2(:,1)=b2(:,1)+1197-146;
b = [b1 ;b2];

%b = [a(2:end,1)+1197-137 a(2:end,2)];
    %a2(2:end,1)+1197-137 a2(2:end,2)
    %a3(2:end,1)+1197-137 a3(2:end,2)];%[b1 ;b2];
%b =a;
h1=figure;
plot(b(:,1),b(:,2),'k');
hold on;
%plot(b(:,1),smooth(b(:,1),b(:,2),0.1*length((b(:,1))),'moving'),'color',[0.9,0,0], 'LineWidth',2);
plot(b(:,1),smooth(b(:,1),b(:,2),100,'moving'),'color',[0.9,0,0], 'LineWidth',2);

sinit = 1118;
%subplot(2,1,3);
%plot([s3(1:sinit,1);s3(sinit+1:end,1)-146],0.3*s3(:,2), 'LineWidth',1,'color','r');

plot_limits=axis;
axis([0, b(end,1), 0, 0.35]);
plot_limits=axis;

grid;
%title('Medium Grid Observation Error');
xlabel('Frame #');
ylabel('per frame RMS (m)');

legend_heigth=0.95;

%inicializción
ini_ini=1;
ini_fin=1200;
plot([ini_fin, ini_fin],[plot_limits(3) plot_limits(4)],'b', 'LineWidth',2);
text((ini_ini+(ini_fin-ini_ini)*0.1), plot_limits(3)+(plot_limits(4)-plot_limits(3))*legend_heigth,sprintf('rigid \ninit.'), 'FontWeight','Bold','color','b');

ini_ini=1200;
ini_fin=5000;
plot([ini_fin, ini_fin],[plot_limits(3) plot_limits(4)],'b', 'LineWidth',2);
text((ini_ini+(ini_fin-ini_ini)*0.4), plot_limits(3)+(plot_limits(4)-plot_limits(3))*legend_heigth,sprintf('deform \nsmooth'), 'FontWeight','Bold','color','b');



ini_ini=5001;
ini_fin=6600;
plot([ini_fin, ini_fin],[plot_limits(3) plot_limits(4)],'b', 'LineWidth',2);
text((ini_ini+(ini_fin-ini_ini)*0.1), plot_limits(3)+(plot_limits(4)-plot_limits(3))*legend_heigth,sprintf('deform \nmedium'), 'FontWeight','Bold','color','b');


ini_ini=6601;
ini_fin=9200;
plot([ini_fin, ini_fin],[plot_limits(3) plot_limits(4)],'b', 'LineWidth',2);
text((ini_ini+(ini_fin-ini_ini)*0.3), plot_limits(3)+(plot_limits(4)-plot_limits(3))*legend_heigth,sprintf('deform \nhard'), 'FontWeight','Bold','color','b');

ini_ini=9201;
ini_fin=11189;
plot([ini_fin, ini_fin],[plot_limits(3) plot_limits(4)],'b', 'LineWidth',2);
text((ini_ini+(ini_fin-ini_ini)*0.1), plot_limits(3)+(plot_limits(4)-plot_limits(3))*legend_heigth*0.98,sprintf('deform \n+\nocclusion'), 'FontWeight','Bold','color','b');

%print(h1,'rms_Node_error_superfine_LN5','-depsc');
