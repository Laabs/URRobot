function closeVrep(vrep,clientID)
    vrep.simxAddStatusbarMessage(clientID,'Close my VREP',vrep.simx_opmode_oneshot);
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
    vrep.simxFinish(clientID);
    vrep.delete();     % call the destructor!
    disp('¹Ø±ÕVREP³É¹¦');
end