function [clientID,vrep,JOINTID,LINKID,SlaveJID,SlaveLID]=robotVrepInit(JONITNAME,LINKNAME)

    vrep=remApi('remoteApi'); 
    vrep.simxFinish(-1); 
    clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
    if (clientID<0)
        JOINTID=zeros(length(JONITNAME),1);
        LINKID=zeros(length(LINKNAME),1);
        disp('Failed connecting to remote API server');  
        vrep=[];
        return;
    end
     while 1
            vrep.simxSetFloatingParameter(clientID,vrep.sim_floatparam_simulation_time_step,0.05,vrep.simx_opmode_oneshot);%设置模拟步长
            recode=vrep.simxSynchronous(clientID,true)
            if(recode==vrep.simx_return_ok)
                disp('motor initialization ......');
               vrep.simxSynchronousTrigger(clientID);
                break;
            else continue;
            end
     end

    vrep.simxAddStatusbarMessage(clientID,'Connection succeeded,six_link6 control',vrep.simx_opmode_oneshot);
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);

    JOINTID=zeros(length(JONITNAME),1);
    LINKID=zeros(length(LINKNAME),1);
    % 获取主机控制句柄
    for i=1:1:length(JONITNAME)
        [~,JOINTID(i)]=vrep.simxGetObjectHandle(clientID,JONITNAME{i},vrep.simx_opmode_blocking);
    end
    
    for i=1:1:length(LINKNAME)
        [~,LINKID(i)]=vrep.simxGetObjectHandle(clientID,LINKNAME{i},vrep.simx_opmode_blocking);
    end
    
    % 初始化关节运行模式
    for i=1:1:length(JONITNAME)
         % 主机初始模式设置
        [~,~]=vrep.simxGetJointPosition(clientID,JOINTID(i),vrep.simx_opmode_streaming);
    end
    for i=1:1:length(LINKNAME)
        [~,~]=vrep.simxGetObjectPosition(clientID,LINKID(i),-1,vrep.simx_opmode_streaming);    
    end
end