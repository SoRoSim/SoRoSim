/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mtimes.h
 *
 * Code generation for function 'mtimes'
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
int32_T b_mtimes(const real_T A_data[], const int32_T A_size[2],
                 const real_T B[6], real_T C_data[]);

void c_mtimes(const real_T A_data[], const int32_T A_size[2],
              const real_T B[36], real_T C_data[], int32_T C_size[2]);

void d_mtimes(const emlrtStack *sp, const real_T A_data[],
              const int32_T A_size[2], const real_T B_data[],
              const int32_T B_size[2], emxArray_real_T *C);

void e_mtimes(const real_T A_data[], const int32_T A_size[2],
              const real_T B_data[], const int32_T B_size[2], real_T C_data[],
              int32_T C_size[2]);

int32_T f_mtimes(const real_T A_data[], const int32_T A_size[2],
                 const real_T B[6], real_T C_data[]);

void g_mtimes(const real_T A_data[], const int32_T A_size[2], real_T C_data[],
              int32_T C_size[2]);

void mtimes(const real_T A_data[], const int32_T A_size[2],
            const real_T B_data[], const int32_T B_size[2], real_T C_data[],
            int32_T C_size[2]);

/* End of code generation (mtimes.h) */
