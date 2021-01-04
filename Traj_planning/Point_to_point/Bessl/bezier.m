% �㵽�㱴�������߹켣�滮������״Σ�
%% ���룺
%   Point(n��1) �ؽڽ�����,n Ϊ·������� 
%% �����
%   retp(T��1) �滮�ؽڽǶ� , TΪ�ܵĹ켣�����
%   retv(T��1)  �滮�ؽڽ��ٶ�
%   reta(T��1)  �滮�ؽڽǼ��ٶ�
%   t(T��1)  ʱ��

function varargout=bezier(Point)% ���Ƶ�/��������
    Point=Point';
    dt=0.01;
    t=0.000001:dt:1;% �����յ�����
    retp=[];retv=[];reta=[];
    tp=[];tv=[];ta=[];
    % ��ʼ��
    P=Point(1,:);
    len=length(P)-1;
    for i=len:-1:0
        temp=(1-t).^i.*t.^(len-i);
        tp=[tp,temp'];%��ϻ�������λ��
        
        %�������
%         syms t i len
%         y=(1-t)^i*t^(len-i)
%         dy=diff(y,t)
%         ddy=diff(dy,t);
%         simplify(ddy)
        tv_tmp=- t.^(len - i - 1).*(i - len).*(1 - t).^i - i*t.^(len - i).*(1 - t).^(i - 1);%��ϻ��������ٶ�
        ta_tmp=2*i*t.^(len - i - 1).*(i - len).*(1 - t).^(i - 1) + i*t.^(len - i).*(i - 1).*(1 - t).^(i - 2) + t.^(len - i - 2).*(i - len).*(1 - t).^i.*(i - len + 1); %��ϻ����������ٶ�
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