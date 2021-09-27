% https://www.mathworks.com/help/fusion/ug/rotations-orientation-and-quaternions.html#d123e7902
% https://www.mathworks.com/help/robotics/ref/quaternion.angvel.html


% generate a sequence of rotations
eulerAngles = [(0:10:90).',zeros(numel(0:10:90),2)];
q = quaternion(eulerAngles,'eulerd','ZYX','frame')

% calculate their angular velocities in between
dt = 1;
av = angvel(q,dt,'frame') % units in rad/s;

% simulate again using the angular velocities
qx = [q(1)];

for i=1:size(av,1)
    w =  av(i,:);
    nw = norm(w);
    vw = [0;0;0];
    if nw ~= 0
        vw = w/nw*sin(nw*dt/2);
    else
        vw = [0;0;0];
    end

    qx =qx*quaternion(cos(nw*dt/2),vw(1),vw(2),vw(3))
end