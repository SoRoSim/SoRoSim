/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_SoftJointDifferentialKinematics_Z2_mex.c
 *
 * Code generation for function '_coder_SoftJointDifferentialKinematics_Z2_mex'
 *
 */

/* Include files */
#include "_coder_SoftJointDifferentialKinematics_Z2_mex.h"
#include "SoftJointDifferentialKinematics_Z2_data.h"
#include "SoftJointDifferentialKinematics_Z2_initialize.h"
#include "SoftJointDifferentialKinematics_Z2_terminate.h"
#include "_coder_SoftJointDifferentialKinematics_Z2_api.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void SoftJointDifferentialKinematics_Z2_mexFunction(int32_T nlhs,
                                                    mxArray *plhs[12],
                                                    int32_T nrhs,
                                                    const mxArray *prhs[6])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs[12];
  int32_T i;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 6) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 6, 4,
                        34, "SoftJointDifferentialKinematics_Z2");
  }
  if (nlhs > 12) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 34,
                        "SoftJointDifferentialKinematics_Z2");
  }
  /* Call the function. */
  c_SoftJointDifferentialKinemati(prhs, nlhs, outputs);
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
  mexAtExit(&SoftJointDifferentialKinematics_Z2_atexit);
  /* Module initialization. */
  SoftJointDifferentialKinematics_Z2_initialize();
  /* Dispatch the entry-point. */
  SoftJointDifferentialKinematics_Z2_mexFunction(nlhs, plhs, nrhs, prhs);
  /* Module termination. */
  SoftJointDifferentialKinematics_Z2_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, "windows-1252", true);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_SoftJointDifferentialKinematics_Z2_mex.c) */
