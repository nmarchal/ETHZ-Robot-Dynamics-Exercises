function I_r_IE = jointToPosition(q)
  % Input: joint angles
  % Output: position of end-effector w.r.t. inertial frame. I_r_IE
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
  %position in the inertial frame
  I_r_IE = T(1:3,4) ;
end