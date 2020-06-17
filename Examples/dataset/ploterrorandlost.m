%% 2 + 0 
function h_plot = ploterrorandlost(filestatus,fileerrorFrame,colorandstyle,num,medianwidth)
lost = load(filestatus);
RMS90 = load(fileerrorFrame);

size_lost=size(lost,1);
size_RMS90=size(RMS90,1);
Alteracionlost = 1;
lostaux = [];
for j=2:size_lost-1
    if(lost(j,1)~=lost(j+1,1)-1)
      FirstX = j;
     % fprintf(1,'Indices no consecutivos en posición %d\n',RMS90(j,1));
      j=j+1;
      LastX = j;
      lostaux = [lostaux;lost(FirstX,:);[(lost(FirstX,1)+1):(lost(LastX,1)-1)]' ones(length((lost(FirstX,1)+1):(lost(LastX,1)-1)),1)];
    else
      lostaux = [lostaux; lost(j,:)];
    end  
end

lost=lostaux;

size_lost=size(lost,1);
for j=1:size_lost-1
    if(lost(j,1)>lost(j+1,1))
      fprintf(1,'Alteración en el orden de los índices en la posición:%d, frame:%d:\n',j,RMS90(j,1));
      Alteracion = j;
    end
end
%addIndexlost=[[(lost(Alteracionlost,1)+1):(lost(Alteracionlost+1,1)-1)]' ones(length([lost(Alteracionlost,1)+1:lost(Alteracionlost+1,1)-1]'),1)];
%lost=[lost(2:Alteracionlost,:);addIndexlost;lost(Alteracionlost+1:end,:)];
    
for j=1:size_RMS90-1
    if(RMS90(j,1)>RMS90(j+1,1))
      fprintf(1,'Alteración en el orden de los índices en la posición:%d, frame:%d:\n',j,RMS90(j,1));
      Alteracion = j;
    end
end

RMS90=[RMS90(1:Alteracion,:);RMS90(Alteracion+1:end,1)+1200 RMS90(Alteracion+1:end,2)];

Xquery = [];
FirstX = 0;
LastX= 0;
RMSaux = [];
for j=1:size_RMS90-1    
    if(RMS90(j,1)~=(RMS90(j+1,1)-1))
      FirstX = j;
     % fprintf(1,'Indices no consecutivos en posición %d\n',RMS90(j,1));
      j=j+1;
      LastX = j;
      Xquery= [RMS90(FirstX,1):RMS90(LastX,1)]';
      vq = interp1(RMS90(FirstX:LastX,1),RMS90(FirstX:LastX,2),Xquery(2:end-1));
      RMSaux = [RMSaux;RMS90(FirstX,:);Xquery(2:end-1) vq];
    else
      RMSaux = [RMSaux; RMS90(j,:)];
    end  
end

 %[RMSaux(:,2), lambda] = regdatasmooth (RMSaux(:,1), RMSaux(:,2), "d",4,"stdev",1e-1,"midpointrule");

RMS90 = [RMSaux(:,1) smooth(RMSaux(:,1),RMSaux(:,2),medianwidth,'moving')];
%RMS90=RMSaux;
for j=1:size_RMS90-1    
    if(RMS90(j,1)~=(RMS90(j+1,1)-1))
      fprintf(1,'Indices no consecutivos en posición %d\n',RMS90(j,1));
    end
end

indices=1:size_RMS90;
indices_lost=lost(:,2)==1;
indices_located=lost(1:length(RMS90),2)==0;

hold on;
h_plot=plot(lost(indices_lost,1),lost(indices_lost,2)*num,colorandstyle,'MarkerSize',4);
h_plot=plot(RMS90(indices_located,1),RMS90(indices_located,2),colorandstyle,'MarkerSize',4);
a= RMS90(indices_located,2);
Mean=mean(a)
Max=max(a)
Min = min(a)
Lenght = length(RMS90(indices_located,2))
end
