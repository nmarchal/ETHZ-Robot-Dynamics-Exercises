function T12 = jointToTransform12(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 2 to frame 1. T_12
  C = rotationY(q(2)) ;
  vector = [0 ; 0 ; 0.145] ;
  T12 = computeTransformationMatrix(C, vector) ;
end