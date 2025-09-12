/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * RigidJointDifferentialKinematics_initialize.c
 *
 * Code generation for function 'RigidJointDifferentialKinematics_initialize'
 *
 */

/* Include files */
#include "RigidJointDifferentialKinematics_initialize.h"
#include "RigidJointDifferentialKinematics_data.h"
#include "_coder_RigidJointDifferentialKinematics_mex.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void RigidJointDifferentialKinematics_once(void);

/* Function Definitions */
static void RigidJointDifferentialKinematics_once(void)
{
  mex_InitInfAndNan();
}

void RigidJointDifferentialKinematics_initialize(void)
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
    RigidJointDifferentialKinematics_once();
  }
}

/* End of code generation (RigidJointDifferentialKinematics_initialize.c) */
