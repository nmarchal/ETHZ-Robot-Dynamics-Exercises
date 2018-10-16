function B_r = rotVecWithQuat(q_BA,A_r)
  % Input: the orientation quaternion and the coordinate of the vector to be mapped
  % Output: the coordinates of the vector in the target frame
  C_AB = quatToRotMat(q_BA);
  B_r = C_AB*A_r;
end
