% https://www.mathworks.com/help/fusion/ug/rotations-orientation-and-quaternions.html#d123e7902
% https://www.mathworks.com/help/robotics/ref/quaternion.angvel.html


q1 = quaternion(1,2,3,4)

eulerAngles = [(0:10:90).',zeros(numel(0:10:90),2)];
q = quaternion(eulerAngles,'eulerd','ZYX','frame');

dt = 1;
av = angvel(q,dt,'frame') % units in rad/s

% simulate again 
qx = [q(1)]