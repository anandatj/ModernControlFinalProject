# ModernControlFinalProject

The DC Motor model with C-Mex is provided in DCMotor.c file. This file needs to be compiled to mex (mexw64) file using MATLAB. 

- DC Motor Performance without controller -> DCMotorMexNoControl.slx
- DC Motor with MPC Speed Controller -> DCMotorMPC.slx
- Testing observability, controllability, and stability of PMDC -> system_test.m
- Testing real time MPC algorithm without simulink -> RealtimeMPC.m
- MPC code used in Simulink -> MPC_RealTime_Simulink.m
