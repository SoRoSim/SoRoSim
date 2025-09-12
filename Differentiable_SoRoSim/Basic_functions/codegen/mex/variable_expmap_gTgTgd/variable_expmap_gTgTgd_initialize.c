/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * variable_expmap_gTgTgd_initialize.c
 *
 * Code generation for function 'variable_expmap_gTgTgd_initialize'
 *
 */

/* Include files */
#include "variable_expmap_gTgTgd_initialize.h"
#include "_coder_variable_expmap_gTgTgd_mex.h"
#include "rt_nonfinite.h"
#include "variable_expmap_gTgTgd_data.h"

/* Function Declarations */
static void variable_expmap_gTgTgd_once(void);

/* Function Definitions */
static void variable_expmap_gTgTgd_once(void)
{
  mex_InitInfAndNan();
}

void variable_expmap_gTgTgd_initialize(void)
{
  static const volatile char_T *emlrtBreakCheckR2012bFlagVar = NULL;
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
    variable_expmap_gTgTgd_once();
  }
}

/* End of code generation (variable_expmap_gTgTgd_initialize.c) */
