/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_compute_dSTdqFC_Z2R_mex.c
 *
 * Code generation for function '_coder_compute_dSTdqFC_Z2R_mex'
 *
 */

/* Include files */
#include "_coder_compute_dSTdqFC_Z2R_mex.h"
#include "_coder_compute_dSTdqFC_Z2R_api.h"
#include "compute_dSTdqFC_Z2R_data.h"
#include "compute_dSTdqFC_Z2R_initialize.h"
#include "compute_dSTdqFC_Z2R_terminate.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void compute_dSTdqFC_Z2R_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                     int32_T nrhs, const mxArray *prhs[7])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 7) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 7, 4,
                        19, "compute_dSTdqFC_Z2R");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 19,
                        "compute_dSTdqFC_Z2R");
  }
  /* Call the function. */
  compute_dSTdqFC_Z2R_api(prhs, &outputs);
  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&compute_dSTdqFC_Z2R_atexit);
  /* Module initialization. */
  compute_dSTdqFC_Z2R_initialize();
  /* Dispatch the entry-point. */
  compute_dSTdqFC_Z2R_mexFunction(nlhs, plhs, nrhs, prhs);
  /* Module termination. */
  compute_dSTdqFC_Z2R_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, "windows-1252", true);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_compute_dSTdqFC_Z2R_mex.c) */
