function T23 = jointToTransform23(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 3 to frame 2. T_23
  C = rotationY(q(3)) ;
  vector = [0 ; 0 ; 0.270] ;
  T23 = computeTransformationMatrix(C, vector) ;
end
