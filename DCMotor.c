#define S_FUNCTION_LEVEL 2 
#define S_FUNCTION_NAME DCMotor
#include "simstruc.h" 
#include <math.h> 

#define U(element) (*uPtrs[element]) /*Pointer to Input Port0*/ 

static void mdlInitializeSizes(SimStruct *S){ 
ssSetNumContStates(S, 2); 
if (!ssSetNumInputPorts(S, 1)) return; 
ssSetInputPortWidth(S, 0, 2); 
ssSetInputPortDirectFeedThrough(S, 0, 1); 
ssSetInputPortOverWritable(S, 0, 1); 
if (!ssSetNumOutputPorts(S, 1)) return; 
ssSetOutputPortWidth(S, 0, 2);
ssSetNumSampleTimes(S, 1); 

ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE); } 

static void mdlInitializeSampleTimes(SimStruct *S) { 
ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME); 
ssSetOffsetTime(S, 0, 0.0); } 

#define MDL_INITIALIZE_CONDITIONS 
static void mdlInitializeConditions(SimStruct *S) { 

real_T *X0 = ssGetContStates(S); 
int_T nStates = ssGetNumContStates(S); 
int_T i; 

/* initialize the states to 0.0 */ 
for (i=0; i < nStates; i++) {X0[i] = 0;} } 

static void mdlOutputs(SimStruct *S, int_T tid) { 
real_T *Y = ssGetOutputPortRealSignal(S,0); 
real_T *X = ssGetContStates(S); 
InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); 

Y[0] = X[1]; //Outputs Current
Y[1] = X[0]*((60/2*3.1415)); //Rad/S into RPM

} 

#define MDL_DERIVATIVES 
static void mdlDerivatives(SimStruct *S) { 
real_T *dX = ssGetdX(S); 
real_T *X = ssGetContStates(S); 
InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); 

real_T Ia_dot,Ia,Vb,La,wr_dot,Te,Tl,b,wr,J,Ra,Kt,Kb,B,Vin;

//Input of Vb and Tl
Vin = U(0);
Tl = U(1);
J	= 0.5215;
Kb	= 1.28;
Kt	= 1.28;
B	= 0.002953;
Ra	= 11.2;
La	= 0.1215;
//Integral of Wr and Ia
wr = X[0];
Ia = X[1];

Vb=Kb*wr;

Ia_dot = (Vin-Ia*Ra-Vb)/La;
Te = Kt*Ia;
wr_dot=(Te-Tl-B*wr)/J;

//Save derivative of wr and Ia
dX[0] = wr_dot;
dX[1] = Ia_dot;
} 

static void mdlTerminate(SimStruct *S) 
{} /*Keep this function empty since no memory is allocated*/ 

#ifdef MATLAB_MEX_FILE 
/* Is this file being compiled as a MEX-file? */ 
#include "simulink.c" /* MEX-file interface mechanism */ 
#else 
#include "cg_sfun.h" /*Code generation registration function*/ 
#endif 