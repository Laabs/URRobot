clc;
clear;
Angle=[0;pi/3;pi/3;pi/4;pi/2];

%贝塞尔曲线
[t,P,V,A]=bezier(Angle);% 机械臂各轴控制点/插入点个数

subplot(311)
plot(P);
drawnow
title('位置曲线')
subplot(312)
plot(V);
title('速度曲线')
subplot(313)
plot(A);
title('加速度曲线')

