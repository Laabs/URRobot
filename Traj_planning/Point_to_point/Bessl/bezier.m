% 点到点贝塞尔曲线轨迹规划（任意阶次）
%% 输入：
%   Point(n×1) 关节角序列,n 为路径点个数 
%% 输出：
%   retp(T×1) 规划关节角度 , T为总的轨迹点个数
%   retv(T×1)  规划关节角速度
%   reta(T×1)  规划关节角加速度
%   t(T×1)  时间

function varargout=bezier(Point)% 控制点/插入点个数
    Point=Point';
    dt=0.01;
    t=0.000001:dt:1;% 起点和终点坐标
    retp=[];retv=[];reta=[];
    tp=[];tv=[];ta=[];
    % 初始化
    P=Point(1,:);
    len=length(P)-1;
    for i=len:-1:0
        temp=(1-t).^i.*t.^(len-i);
        tp=[tp,temp'];%混合基函数，位置
        
        %符号求解
%         syms t i len
%         y=(1-t)^i*t^(len-i)
%         dy=diff(y,t)
%         ddy=diff(dy,t);
%         simplify(ddy)
        tv_tmp=- t.^(len - i - 1).*(i - len).*(1 - t).^i - i*t.^(len - i).*(1 - t).^(i - 1);%混合基函数，速度
        ta_tmp=2*i*t.^(len - i - 1).*(i - len).*(1 - t).^(i - 1) + i*t.^(len - i).*(i - 1).*(1 - t).^(i - 2) + t.^(len - i - 2).*(i - len).*(1 - t).^i.*(i - len + 1); %混合基函数，加速度
        tv=[tv,tv_tmp'];
        ta=[ta,ta_tmp'];
    end
    for id=1:1:length(Point(:,1))  
        P=Point(id,:);
        yp=0;
        yv=0;
        ya=0;
        for i=1:length(P)
            yp=yp+P(i)*tp(:,i);
            yv=yv+P(i)*tv(:,i);
            ya=ya+P(i)*ta(:,i);
        end
        retp=[retp,yp];
        retv=[retv,yv];
        reta=[reta,ya];
    end
     
    varargout{1}=retp;  
    varargout{2}=retv;  
    varargout{3}=reta;  
    varargout{4}=t'; 
end