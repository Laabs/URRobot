clc;
clear;
Angle=[0;pi/3;pi/3;pi/4;pi/2];

%����������
[t,P,V,A]=bezier(Angle);% ��е�۸�����Ƶ�/��������

subplot(311)
plot(P);
drawnow
title('λ������')
subplot(312)
plot(V);
title('�ٶ�����')
subplot(313)
plot(A);
title('���ٶ�����')

