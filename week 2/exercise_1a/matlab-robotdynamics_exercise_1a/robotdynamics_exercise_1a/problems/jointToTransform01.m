function T01 = jointToTransform01(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 1 to frame 0. T_01
  C = rotationZ(q(1)) ;
  vector = [0 ; 0 ; 0.145] ;
  T01 = computeTransformationMatrix(C, vector) ;
end