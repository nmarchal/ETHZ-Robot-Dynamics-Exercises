function R = quatToRotMat(q)
  % Input: quaternion [w x y z]
  % Output: corresponding rotation matrix
  q_im = q(2:4,1);
  R= (2*(q(1)^2) -1) * eye(3) + 2*q(1) *crossProdMat(q_im) + 2 * q_im * q_im' ;
end
