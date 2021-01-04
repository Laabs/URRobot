clc;
clear all;
addpath(genpath('.'));
% ���
JONITNAME={'UR10_joint1','UR10_joint2','UR10_joint3','UR10_joint4','UR10_joint5','UR10_joint6'};%���� 6 ���ؽ�����
LINKNAME={'UR10_end'};%������������
[clientID,vrep,JOINTID,LINKID]=robotVrepInit(JONITNAME,LINKNAME);

STEP=100;
for i=0:1:STEP
    theta=[i/STEP*0.5*pi;i/STEP*0.3*pi;i/STEP*0.5*pi;i/STEP*0.4*pi;i/STEP*0.6*pi;i/STEP*0.2*pi];
    vtheta=[0.5;0.5;0.5;0.5;0.5;0.5];
    vreptheta=zeros(1,6);
    vreplink=zeros(1,3);
    
%% 1.��ȡ����λ��
if 1
    for j=1:1:length(LINKID)
        [~,vreplink(j,:)]=vrep.simxGetObjectPosition(clientID,LINKID(j),-1,vrep.simx_opmode_oneshot);     % ��ȡĩ��λ��
    end   
end
    
%% 2.��ȡ�ؽ�λ��   
if 1
    for j=1:1:length(JOINTID)
        [~,vreptheta(1,j)]=vrep.simxGetJointPosition(clientID,JOINTID(j),vrep.simx_opmode_oneshot);% �����ؽڽ�λ��
    end    
end

%% 3.λ��ģʽ
if 1
    for j=1:1:length(JOINTID)
        vrep.simxSetJointTargetPosition (clientID,JOINTID(j),theta(j),vrep.simx_opmode_oneshot);
    end
end

%% 4.�ٶ�ģʽ
if 0
    for j=1:1:6
        vrep.simxSetJointTargetVelocity(clientID,JOINTID(j),vtheta(j),vrep.simx_opmode_oneshot);
    end
end
    vrep.simxSynchronousTrigger(clientID);
end
closeVrep(vrep,clientID);
