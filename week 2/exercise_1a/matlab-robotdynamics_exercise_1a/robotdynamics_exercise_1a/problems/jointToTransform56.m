function T56 = jointToTransform56(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 6 to frame 5. T_56
  C = rotationX(q(6)) ;
  vector = [0.072 ; 0 ; 0] ;
  T56 = computeTransformationMatrix(C, vector) ;
end
