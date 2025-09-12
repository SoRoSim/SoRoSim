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
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void b_mtimes(const real_T A[36], const real_T B_data[],
              const int32_T B_size[2], real_T C_data[], int32_T C_size[2]);

void mtimes(const real_T A_data[], const int32_T A_size[2],
            const real_T B_data[], int32_T B_size, real_T C[6]);

/* End of code generation (mtimes.h) */
