times0 = load('strech/nonfixedcam/2+0/2/time.txt');
times1 = load('strech/nonfixedcam/2+1/5/time.txt');
times2 = load('strech/nonfixedcam/2+2/2/time.txt');
%times3 = load('strech/nonfixedcam/2+3/time.txt');
timeslocal0 = load('strech/nonfixedcam/2+0/3/localtime.txt');
timeslocal1 = load('strech/nonfixedcam/2+1/5/localtime.txt');
timeslocal2 = load('strech/nonfixedcam/2+2/2/localtime.txt');
%timeslocal3 = load('strech/nonfixedcam/2+3/localtime.txt');
length(times0);
mean(times0(1141:1140+length(timeslocal0)-1,2)- timeslocal0(2:end,2))
mean(timeslocal0(2:end,2))
timesGlob0 = [mean((times0(1141:1141+length(timeslocal0(2:end,2))-1,2))-timeslocal0(2:end,2)) mean(timeslocal0(2:end,2))]
timesGlob1 = [mean((times1(1141:1141+length(timeslocal1(2:end,2))-1,2))-timeslocal1(2:end,2)) mean(timeslocal1(2:end,2))]
timesGlob2 = [mean((times2(1142:1142+length(timeslocal2(2:end,2))-1,2))-timeslocal2(2:end,2)) mean(timeslocal2(2:end,2))]
%timesGlob3 = [mean((times3(1141:1141+length(timeslocal3(2:end,2))-1,2))-timeslocal3(2:end,2)) mean(timeslocal3(2:end,2))]

