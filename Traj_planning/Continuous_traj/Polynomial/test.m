clc;clear;

th=[-3.3;-2.0;-81.5;83.5;83.4;90];%�ؽڽǶ�
t=(linspace(0,4,length(th)))';%ʱ��

[thseq5,vseq5,aseq5,tseq5]=fifthInperp(t,th);
[thseq3,vseq3,aseq3,tseq3]=threeInperp(t,th);

figure(1)
subplot(311)
hold on;
plot(tseq3,thseq3,'k');
plot(tseq5,thseq5,'r');
plot(t,th,'rp');
title('λ��');
subplot(312)
hold on;
plot(tseq3,vseq3,'k');
plot(tseq5,vseq5,'r');
title('�ٶ�');
subplot(313)
hold on;
plot(tseq3,aseq3,'k');
plot(tseq5,aseq5,'r');
title('���ٶ�');




