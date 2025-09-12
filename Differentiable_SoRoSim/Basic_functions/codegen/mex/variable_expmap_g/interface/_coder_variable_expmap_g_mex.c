/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_variable_expmap_g_mex.c
 *
 * Code generation for function '_coder_variable_expmap_g_mex'
 *
 */

/* Include files */
#include "_coder_variable_expmap_g_mex.h"
#include "_coder_variable_expmap_g_api.h"
#include "rt_nonfinite.h"
#include "variable_expmap_g_data.h"
#include "variable_expmap_g_initialize.h"
#include "variable_expmap_g_terminate.h"

/* Function Definitions */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&variable_expmap_g_atexit);
  /* Module initialization. */
  variable_expmap_g_initialize();
  /* Dispatch the entry-point. */
  variable_expmap_g_mexFunction(nlhs, plhs, nrhs, prhs);
  /* Module termination. */
  variable_expmap_g_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, "windows-1252", true);
  return emlrtRootTLSGlobal;
}

void variable_expmap_g_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs,
                                   const mxArray *prhs[1])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 1, 4,
                        17, "variable_expmap_g");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 17,
                        "variable_expmap_g");
  }
  /* Call the function. */
  variable_expmap_g_api(prhs[0], &outputs);
  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

/* End of code generation (_coder_variable_expmap_g_mex.c) */
