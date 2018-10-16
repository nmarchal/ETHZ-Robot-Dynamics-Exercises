clc
clear all
close all

C_IE_des = jointToRotMat (pi/4 * ones(6,1)) ;
q_0 = 0*ones(6,1) ;
tol = 0.01 ;
I_r_IE_des = jointToPosition(pi/8 * ones(6,1)) ;

inverseKinematics(I_r_IE_des, C_IE_des, q_0, tol) ;