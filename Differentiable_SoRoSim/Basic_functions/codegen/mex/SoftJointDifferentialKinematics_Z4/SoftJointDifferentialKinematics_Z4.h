/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * SoftJointDifferentialKinematics_Z4.h
 *
 * Code generation for function 'SoftJointDifferentialKinematics_Z4'
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
void SoftJointDifferentialKinematics_Z4(
    const emlrtStack *sp, real_T h, const real_T Phi_Z1_data[],
    const int32_T Phi_Z1_size[2], const real_T Phi_Z2_data[],
    const int32_T Phi_Z2_size[2], const real_T xi_star_Z1[6],
    const real_T xi_star_Z2[6], const real_T q_data[], const int32_T q_size[1],
    const real_T qd_data[], const int32_T qd_size[1], const real_T qdd_data[],
    const int32_T qdd_size[1], real_T Omega[6], real_T Z_data[],
    int32_T Z_size[2], real_T g[16], real_T T[36], real_T S_data[],
    int32_T S_size[2], real_T Sd_data[], int32_T Sd_size[2], real_T f[4],
    real_T fd[4], real_T adjOmegap[144], real_T dSdq_qd_data[],
    int32_T dSdq_qd_size[2], real_T dSdq_qdd_data[], int32_T dSdq_qdd_size[2],
    real_T dSddq_qd_data[], int32_T dSddq_qd_size[2]);

/* End of code generation (SoftJointDifferentialKinematics_Z4.h) */
