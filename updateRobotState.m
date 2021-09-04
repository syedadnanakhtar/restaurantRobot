function newState = updateRobotState(oldState, vel, omega, timestep)
    angle = omega*timestep;
    if omega ~= 0
        radius = vel/omega;            
        deltaX = radius*sin(angle);
        deltaY = radius*(1-cos(angle));
    else
        deltaX = vel*timestep;
        deltaY = 0;
    end
    deltaXGlobal = cos(oldState.heading)*deltaX - sin(oldState.heading)*deltaY;
    deltaYGlobal = sin(oldState.heading)*deltaX + cos(oldState.heading)*deltaY;
    newState.x = round(oldState.x + deltaXGlobal,2);%oldState.x + deltaXGlobal;
    newState.y = round(oldState.y + deltaYGlobal,2);%oldState.y + deltaYGlobal;
    newState.heading = oldState.heading + angle;
    newState.vel = vel;
    newState.omega = omega;
   
    if abs(newState.heading - pi/2) <= 0.01
        newState.heading = pi/2;
    elseif abs(newState.heading + pi/2) <= 0.01
        newState.heading = -pi/2;
    elseif abs(newState.heading) <= 0.01
        newState.heading = 0;
    end
end