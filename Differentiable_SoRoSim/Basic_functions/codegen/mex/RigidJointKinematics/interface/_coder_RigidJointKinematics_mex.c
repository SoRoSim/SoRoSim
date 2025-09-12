/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_RigidJointKinematics_mex.c
 *
 * Code generation for function '_coder_RigidJointKinematics_mex'
 *
 */

/* Include files */
#include "_coder_RigidJointKinematics_mex.h"
#include "RigidJointKinematics_data.h"
#include "RigidJointKinematics_initialize.h"
#include "RigidJointKinematics_terminate.h"
#include "_coder_RigidJointKinematics_api.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void RigidJointKinematics_mexFunction(int32_T nlhs, mxArray *plhs[8],
                                      int32_T nrhs, const mxArray *prhs[3])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs[8];
  int32_T i;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 3, 4,
                        20, "RigidJointKinematics");
  }
  if (nlhs > 8) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 20,
                        "RigidJointKinematics");
  }
  /* Call the function. */
  RigidJointKinematics_api(prhs, nlhs, outputs);
  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    i = 1;
  } else {
    i = nlhs;
  }
  emlrtReturnArrays(i, &plhs[0], &outputs[0]);
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&RigidJointKinematics_atexit);
  /* Module initialization. */
  RigidJointKinematics_initialize();
  /* Dispatch the entry-point. */
  RigidJointKinematics_mexFunction(nlhs, plhs, nrhs, prhs);
  /* Module termination. */
  RigidJointKinematics_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, "windows-1252", true);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_RigidJointKinematics_mex.c) */
