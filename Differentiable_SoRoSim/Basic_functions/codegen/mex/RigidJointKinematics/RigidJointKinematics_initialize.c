/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * RigidJointKinematics_initialize.c
 *
 * Code generation for function 'RigidJointKinematics_initialize'
 *
 */

/* Include files */
#include "RigidJointKinematics_initialize.h"
#include "RigidJointKinematics_data.h"
#include "_coder_RigidJointKinematics_mex.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void RigidJointKinematics_once(void);

/* Function Definitions */
static void RigidJointKinematics_once(void)
{
  mex_InitInfAndNan();
}

void RigidJointKinematics_initialize(void)
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
    RigidJointKinematics_once();
  }
}

/* End of code generation (RigidJointKinematics_initialize.c) */
