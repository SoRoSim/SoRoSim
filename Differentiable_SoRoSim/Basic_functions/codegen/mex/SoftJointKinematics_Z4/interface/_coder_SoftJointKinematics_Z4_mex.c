/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_SoftJointKinematics_Z4_mex.c
 *
 * Code generation for function '_coder_SoftJointKinematics_Z4_mex'
 *
 */

/* Include files */
#include "_coder_SoftJointKinematics_Z4_mex.h"
#include "SoftJointKinematics_Z4_data.h"
#include "SoftJointKinematics_Z4_initialize.h"
#include "SoftJointKinematics_Z4_terminate.h"
#include "_coder_SoftJointKinematics_Z4_api.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void SoftJointKinematics_Z4_mexFunction(int32_T nlhs, mxArray *plhs[8],
                                        int32_T nrhs, const mxArray *prhs[6])
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
  if (nrhs != 6) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 6, 4,
                        22, "SoftJointKinematics_Z4");
  }
  if (nlhs > 8) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 22,
                        "SoftJointKinematics_Z4");
  }
  /* Call the function. */
  SoftJointKinematics_Z4_api(prhs, nlhs, outputs);
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
  mexAtExit(&SoftJointKinematics_Z4_atexit);
  /* Module initialization. */
  SoftJointKinematics_Z4_initialize();
  /* Dispatch the entry-point. */
  SoftJointKinematics_Z4_mexFunction(nlhs, plhs, nrhs, prhs);
  /* Module termination. */
  SoftJointKinematics_Z4_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, "windows-1252", true);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_SoftJointKinematics_Z4_mex.c) */
