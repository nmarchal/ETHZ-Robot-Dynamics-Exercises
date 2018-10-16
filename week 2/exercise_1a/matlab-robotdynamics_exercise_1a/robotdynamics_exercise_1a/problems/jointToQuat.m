function quat = jointToQuat(q)
  % Input: joint angles
  % Output: quaternion representing the orientation of the end-effector
  % q_IE.
  R = jointToRotMat(q) ;
  quat = rotMatToQuat(R) ;
end