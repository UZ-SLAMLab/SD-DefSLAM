close all
h1 = figure;

ymax=0.15;
axis([0 11189 0 ymax]);
num=ymax-0.02;
medianwith=200;
grid on;
%h_plot12 = ploterrorandlost('/home/jose/ORB-SDTAM/Examples/simulation/status.txt','/home/jose/ORB-SDTAM/Examples/simulation/errorframe.txt','.k',0.0045,medianwith);
h_plot0 = ploterrorandlost('strech/nonfixedcam/2+0/2/status.txt','strech/nonfixedcam/2+0/2/errorframe.txt','.r',0.0035,medianwith);
h_plot1 = ploterrorandlost('strech/nonfixedcam/2+1/5/status.txt','strech/nonfixedcam/2+1/5/errorframe.txt','.b',0.0025,medianwith);
h_plot2 = ploterrorandlost('strech/nonfixedcam/2+2/2/status.txt','strech/nonfixedcam/2+2/2/errorframe.txt','.g',0.0015,medianwith);
h_plot3 = ploterrorandlost('strech/nonfixedcam/2+3/status.txt','strech/nonfixedcam/2+3/errorframe.txt','.k',0.0005,medianwith);
%h_plot0 = ploterrorandlost('/home/jose/ORB-SDTAM/Examples/dataset/status.txt','/home/jose/ORB-SDTAM/Examples/dataset/errorframe.txt','.r',0+0.0035,medianwith);

%h_plot0 = ploterrorandlost('/home/jose/ORB-SDTAM/Examples/Monocular/Local/0/2/status.txt','/home/jose/ORB-SDTAM/Examples/Monocular/Local/0/2/errorframe.txt','.r',0+0.0035,medianwith);
%h_plot1 = ploterrorandlost('/home/jose/ORB-SDTAM/Examples/Monocular/Local/1/1/status.txt','/home/jose/ORB-SDTAM/Examples/Monocular/Local/1/1/errorframe.txt','.b',0+0.0025,medianwith);
%h_plot2 = ploterrorandlost('/home/jose/ORB-SDTAM/Examples/Monocular/Local/2/1/status.txt','/home/jose/ORB-SDTAM/Examples/Monocular/Local/2/1/errorframe.txt','.g',0+0.0015,medianwith);

%h_plot1 = ploterrorandlost('/home/jose/ORB-SDTAM/Examples/Monocular/Local/2/status.txt','/home/jose/ORB-SDTAM/Examples/Monocular/Local/2/errorframe.txt','.k',0+0.0025,medianwith);
%h_plot2 = ploterrorandlost('/home/jose/ORB-SDTAM/Examples/Monocular/Local/2/2/status.txt','/home/jose/ORB-SDTAM/Examples/Monocular/Local/2/2/errorframe.txt','.k',0+0.0015,medianwith);

%h_plot3 = ploterrorandlost('/home/jose/ORB-SDTAM/Examples/Monocular/Local/3/1/status.txt','/home/jose/ORB-SDTAM/Examples/Monocular/Local/3/1/errorframe.txt','.k',0+0.0005,medianwith);
FontSize = 14;

t11 = text(100,0.1455,'Init','Interpreter','latex','FontSize',FontSize);


xline = 1200;
a0 = line([xline xline],ylim);

t0 = text(xline+100,0.1455,'Smooth','Interpreter','latex','FontSize',FontSize);
xline = 5100;
a1 = line([xline xline],ylim);

t = text(xline+100,0.1455,'Medium','Interpreter','latex','FontSize',FontSize);
xline = 6600
a2 = line([xline xline],ylim);

t1 = text(xline+100,0.1455,'Hard','Interpreter','latex','FontSize',FontSize);
xline = 9500
a3 = line([xline xline],ylim);
legend([h_plot0,h_plot1,h_plot2,h_plot3,a0,a1,a2,a3],'2+0','2+1','2+2','2+3');


t2 = text(xline+100,0.1455,'Oclussion','Interpreter','latex','FontSize',FontSize);
ylabel('RMS error(m)');
xlabel('Frame #');

print(h1,'ErrorLocalComp01','-depsc');
