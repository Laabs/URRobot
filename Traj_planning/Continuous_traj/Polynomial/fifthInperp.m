% 五次多项式轨迹规划
%% 输入：
%   t(1×1) 时间序列
%   th(n×1) 关节角序列,n 为路径点的个数
%% 输出：
% thseq(T×1) 规划关节角度；vseq(T×1) 规划关节角速度；aseq(T×1) 规划关节角加速度；tseq(T×1)
% 生成的时间序列,T为总的轨迹点个数

function [thseq,vseq,aseq,tseq]=fifthInperp(t,th)

flag_0=1;% 初始结束点速度为0 ？
dt = 0.01;% 控制周期
% 速度初始化
v=zeros(length(th),1);
a=zeros(length(th),1);
for i=1:1:length(th)-1
    if i==1
        v(i)=(th(i+1)-th(i))/(t(i+1)-t(i));
    else
        v(i)=((th(i)-th(i-1))/(t(i)-t(i-1))+(th(i+1)-th(i))/(t(i+1)-t(i)))/2;
    end
end
v(length(th))=(th(end)-th(end-1))/(t(end)-t(end-1));
% 加速度初始化
for i=1:1:length(v)-1
    if i==1
        a(i)=(v(i+1)-v(i))/(t(i+1)-t(i));
    else
        a(i)=((v(i)-v(i-1))/(t(i)-t(i-1))+(v(i+1)-v(i))/(t(i+1)-t(i)))/2;
    end
end
a(length(th))=(v(end)-v(end-1))/(t(end)-t(end-1));
if flag_0
    a(end)=0;
    v(end)=0;
    a(1)=0;
    v(1)=0;
end

tseq=[];thseq=[];vseq=[];aseq=[];%时间序列
a0seq=[];a1seq=[];a2seq=[];a3seq=[];a4seq=[];a5seq=[];
for i=1:1:length(t)-1
    t0=t(i);
    P0=th(i);
    V0=v(i);
    A0=a(i);
    t1=t(i+1);
    P1=th(i+1);
    V1=v(i+1);
    A1=a(i+1);
    a0=P0;
    a1=V0;
    a2=A0/2;
%     a3 = (20.0 * (P1-P0) - (8.0 * V1 + 12.0 * V0) * (t1-t0) - (3.0 * A0 - A1) * (t1-t0)^2) / (2.0 * (t1-t0)^3);
%     a4 = (-30.0 * (P1-P0) + (14.0 * V1 + 16.0 * V0) * (t1-t0) + (3.0 * A0 - 2.0 * A1) * (t1-t0)^2) / (2.0 * (t1-t0)^4);
%     a5 = (12.0 * (P1-P0) - 6.0 * (V1 + V0) * (t1-t0) - (A0 - A1) * (t1-t0)^2) / (2.0 * (t1-t0)^5);
    a3=(20*P0 - 20*P1 - 12*V0*t0 + 12*V0*t1 - 8*V1*t0 + 8*V1*t1 + 3*A0*t0^2 + 3*A0*t1^2 - A1*t0^2 - A1*t1^2 - 6*A0*t0*t1 + 2*A1*t0*t1)/(2*(t0 - t1)^3);
    a4=(30*P0 - 30*P1 - 16*V0*t0 + 16*V0*t1 - 14*V1*t0 + 14*V1*t1 + 3*A0*t0^2 + 3*A0*t1^2 - 2*A1*t0^2 - 2*A1*t1^2 - 6*A0*t0*t1 + 4*A1*t0*t1)/(2*(t0 - t1)^4);
    a5=(12*P0 - 12*P1 - 6*V0*t0 + 6*V0*t1 - 6*V1*t0 + 6*V1*t1 + A0*t0^2 + A0*t1^2 - A1*t0^2 - A1*t1^2 - 2*A0*t0*t1 + 2*A1*t0*t1)/(2*(t0 - t1)^5);

   

    a0seq=[a0seq,a0];a1seq=[a1seq,a1];a2seq=[a2seq,a2];a3seq=[a3seq,a3];a4seq=[a4seq,a4];a5seq=[a5seq,a5];
    end

    tseq=(t(1):dt:t(end))';
    idx=1;
    for i=1:1:length(tseq)
        if tseq(i)>=t(idx)
            idx=idx+1;
            if idx <= length(t) 
                a0=a0seq(idx-1);
                a1=a1seq(idx-1);
                a2=a2seq(idx-1);
                a3=a3seq(idx-1);
                a4=a4seq(idx-1);
                a5=a5seq(idx-1);
                t0=t(idx-1);
            end
        end
        thseq=[thseq;a0+a1*(tseq(i)-t0)+a2*(tseq(i)-t0)^2+a3*(tseq(i)-t0)^3+a4*(tseq(i)-t0)^4+a5*(tseq(i)-t0)^5];
        vseq=[vseq;a1 + a2*(2*tseq(i) - 2*t0) + 3*a3*(tseq(i) - t0)^2 + 4*a4*(tseq(i) - t0)^3 + 5*a5*(tseq(i) - t0)^4];% diff(P,t)
        aseq=[aseq;2*a2 + 3*a3*(2*tseq(i) - 2*t0) + 12*a4*(tseq(i) - t0)^2 + 20*a5*(tseq(i) - t0)^3]; % diff(V,t)
    end
end