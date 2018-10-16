function C_IE = jointToRotMat(q)
  % Input: joint angles
  % Output: rotation matrix which projects a vector defined in the
  % end-effector frame E to the inertial frame I, C_IE.
  TI0 = getTransformI0() ;
  T01 = jointToTransform01(q) ;
  T12 = jointToTransform12(q) ;
  T23 = jointToTransform23(q) ;
  T34 = jointToTransform34(q) ;
  T45 = jointToTransform45(q) ;
  T56 = jointToTransform56(q) ;
  T6E = getTransform6E() ;
  %transformation from base to end effector
  T = TI0 * T01 * T12 * T23 * T34 * T45 * T56 * T6E ;
  %rotation matrix from inertial frame to end effector
  C_IE = T(1:3,1:3) ;
end