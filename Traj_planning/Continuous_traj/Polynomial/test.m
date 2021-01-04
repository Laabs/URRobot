clc;clear;

th=[-3.3;-2.0;-81.5;83.5;83.4;90];%关节角度
t=(linspace(0,4,length(th)))';%时间

[thseq5,vseq5,aseq5,tseq5]=fifthInperp(t,th);
[thseq3,vseq3,aseq3,tseq3]=threeInperp(t,th);

figure(1)
subplot(311)
hold on;
plot(tseq3,thseq3,'k');
plot(tseq5,thseq5,'r');
plot(t,th,'rp');
title('位置');
subplot(312)
hold on;
plot(tseq3,vseq3,'k');
plot(tseq5,vseq5,'r');
title('速度');
subplot(313)
hold on;
plot(tseq3,aseq3,'k');
plot(tseq5,aseq5,'r');
title('加速度');




