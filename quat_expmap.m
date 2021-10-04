function out_q = quat_expmap(q, theta)
% this function takes in 
%  q  - matlab quaternion
%  theta - 3x1 vector
%  update q = q\otimes exp(theta)

angle = norm(theta);
axis = theta/angle;

dq = quaternion(cos(angle/2),sin(angle/2)*axis(1),sin(angle/2)*axis(2),sin(angle/2)*axis(3));

out_q = q*dq;

end