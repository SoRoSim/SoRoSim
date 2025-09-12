/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * SoftJointKinematics_Z2_initialize.c
 *
 * Code generation for function 'SoftJointKinematics_Z2_initialize'
 *
 */

/* Include files */
#include "SoftJointKinematics_Z2_initialize.h"
#include "SoftJointKinematics_Z2_data.h"
#include "_coder_SoftJointKinematics_Z2_mex.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void SoftJointKinematics_Z2_once(void);

/* Function Definitions */
static void SoftJointKinematics_Z2_once(void)
{
  mex_InitInfAndNan();
}

void SoftJointKinematics_Z2_initialize(void)
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
    SoftJointKinematics_Z2_once();
  }
}

/* End of code generation (SoftJointKinematics_Z2_initialize.c) */
