/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_compute_dSTdqFC_Z4_mex.c
 *
 * Code generation for function '_coder_compute_dSTdqFC_Z4_mex'
 *
 */

/* Include files */
#include "_coder_compute_dSTdqFC_Z4_mex.h"
#include "_coder_compute_dSTdqFC_Z4_api.h"
#include "compute_dSTdqFC_Z4_data.h"
#include "compute_dSTdqFC_Z4_initialize.h"
#include "compute_dSTdqFC_Z4_terminate.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void compute_dSTdqFC_Z4_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                    int32_T nrhs, const mxArray *prhs[10])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 10) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 10, 4,
                        18, "compute_dSTdqFC_Z4");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 18,
                        "compute_dSTdqFC_Z4");
  }
  /* Call the function. */
  compute_dSTdqFC_Z4_api(prhs, &outputs);
  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&compute_dSTdqFC_Z4_atexit);
  /* Module initialization. */
  compute_dSTdqFC_Z4_initialize();
  /* Dispatch the entry-point. */
  compute_dSTdqFC_Z4_mexFunction(nlhs, plhs, nrhs, prhs);
  /* Module termination. */
  compute_dSTdqFC_Z4_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, "windows-1252", true);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_compute_dSTdqFC_Z4_mex.c) */
