% https://www.mathworks.com/help/fusion/ug/rotations-orientation-and-quaternions.html#d123e7902
% https://www.mathworks.com/help/robotics/ref/quaternion.angvel.html


% generate a sequence of rotations
eulerAngles = [(0:10:90).',(0:5:45).',(0:5:45).'];
q = quaternion(eulerAngles,'eulerd','ZYX','frame')

nt = max(size(q));

av = zeros(3,nt);
for i=2:nt
    dq = q(i-1)'*q(i);
    dt = 0.05;
    [w,x,y,z] = parts(dq);
    nw = acos(w)/dt*2;
    omega = [x;y;z]*nw/sin(nw*dt/2);
    av(:,i-1) = omega;
end
av

% % simulate again using the angular velocities
qx = [q(1)];
for i=1:size(av,2)
    w =  av(:,i);
    nw = norm(w);
    vw = [0;0;0];
    if nw ~= 0
        vw = w/nw*sin(nw*dt/2);
    else
        vw = [0;0;0];
    end
%     if i == size(av,1)
        qx =qx*quaternion(cos(nw*dt/2),vw(1),vw(2),vw(3));
%     else
%         t_v = dt^dt/24*cross(av(:,i),av(:,i+1));
%         qx =qx*(quaternion(cos(nw*dt/2),vw(1),vw(2),vw(3)) + quaternion(0,t_v(1),t_v(2),t_v(3)));
%     end
end
qx
q(end)