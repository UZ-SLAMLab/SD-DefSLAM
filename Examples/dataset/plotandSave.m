close all
h1=figure('visible','off'); 
ymax=0.15;
axis([0 11189 0 ymax]);
num=ymax-0.02;
FontSize = 10;
medianwith=200;

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
%legend([h_plot0,h_plot1,h_plot2,h_plot3,a0,a1,a2,a3],'2+0','2+1','2+2','2+3');


t2 = text(xline+100,0.1455,'Oclussion','Interpreter','latex','FontSize',FontSize);
ylabel('RMS error(m)');
xlabel('Frame #');

h_plot2 = Saveploterrorandlost('strech/nonfixedcam/2+2/2/status.txt','strech/nonfixedcam/2+2/2/errorframe.txt','.r',0.0015,medianwith,h1);