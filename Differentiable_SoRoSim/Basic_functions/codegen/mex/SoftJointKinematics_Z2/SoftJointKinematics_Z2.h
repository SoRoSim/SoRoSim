/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * SoftJointKinematics_Z2.h
 *
 * Code generation for function 'SoftJointKinematics_Z2'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void SoftJointKinematics_Z2(const emlrtStack *sp, real_T h,
                            const real_T Phi_Z_data[],
                            const int32_T Phi_Z_size[2],
                            const real_T xi_star[6], const real_T q_data[],
                            const int32_T q_size[1], real_T Omega[6],
                            real_T Z_data[], int32_T Z_size[2], real_T g[16],
                            real_T T[36], real_T S_data[], int32_T S_size[2],
                            real_T f[4], real_T fd[4], real_T adjOmegap[144]);

/* End of code generation (SoftJointKinematics_Z2.h) */
