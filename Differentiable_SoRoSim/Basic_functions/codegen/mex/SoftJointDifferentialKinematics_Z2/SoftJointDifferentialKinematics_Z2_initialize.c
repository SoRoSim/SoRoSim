/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * SoftJointDifferentialKinematics_Z2_initialize.c
 *
 * Code generation for function 'SoftJointDifferentialKinematics_Z2_initialize'
 *
 */

/* Include files */
#include "SoftJointDifferentialKinematics_Z2_initialize.h"
#include "SoftJointDifferentialKinematics_Z2_data.h"
#include "_coder_SoftJointDifferentialKinematics_Z2_mex.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void SoftJointDifferentialKinematics_Z2_once(void);

/* Function Definitions */
static void SoftJointDifferentialKinematics_Z2_once(void)
{
  mex_InitInfAndNan();
}

void SoftJointDifferentialKinematics_Z2_initialize(void)
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
    SoftJointDifferentialKinematics_Z2_once();
  }
}

/* End of code generation (SoftJointDifferentialKinematics_Z2_initialize.c) */
