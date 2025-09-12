/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compute_dSTdqFC_Z2R.h
 *
 * Code generation for function 'compute_dSTdqFC_Z2R'
 *
 */

#pragma once

/* Include files */
#include "compute_dSTdqFC_Z2R_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void compute_dSTdqFC_Z2R(const emlrtStack *sp, real_T ndof,
                         const real_T Omega[6], const real_T Z_data[],
                         const int32_T Z_size[2], const real_T f[4],
                         const real_T fd[4], const real_T adjOmegap[144],
                         const real_T F_C[6], emxArray_real_T *dSTdq_FC);

/* End of code generation (compute_dSTdqFC_Z2R.h) */
