/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compute_dSTdqFC_Z2R_initialize.c
 *
 * Code generation for function 'compute_dSTdqFC_Z2R_initialize'
 *
 */

/* Include files */
#include "compute_dSTdqFC_Z2R_initialize.h"
#include "_coder_compute_dSTdqFC_Z2R_mex.h"
#include "compute_dSTdqFC_Z2R_data.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void compute_dSTdqFC_Z2R_once(void);

/* Function Definitions */
static void compute_dSTdqFC_Z2R_once(void)
{
  mex_InitInfAndNan();
}

void compute_dSTdqFC_Z2R_initialize(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2022b(&st);
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    compute_dSTdqFC_Z2R_once();
  }
}

/* End of code generation (compute_dSTdqFC_Z2R_initialize.c) */
