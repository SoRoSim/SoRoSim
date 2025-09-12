/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * eml_mtimes_helper.h
 *
 * Code generation for function 'eml_mtimes_helper'
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
void binary_expand_op_13(real_T in1_data[], int32_T in1_size[2],
                         const real_T in2_data[], const int32_T *in2_size,
                         const real_T in3_data[], const int32_T in3_size[2],
                         const real_T in4_data[], const int32_T in4_size[2]);

int32_T binary_expand_op_14(real_T in1_data[], const emlrtRSInfo in2,
                            real_T in3, const real_T in4_data[],
                            const int32_T in4_size[2], const real_T in5_data[],
                            const int32_T in5_size[2], const real_T in6[6]);

void binary_expand_op_15(real_T in1_data[], const emlrtRSInfo in2,
                         const real_T in3[36], real_T in4,
                         const real_T in5_data[], const int32_T in5_size[2],
                         const real_T in6_data[], const int32_T in6_size[2],
                         int32_T in1_size[2]);

void binary_expand_op_16(real_T in1_data[], int32_T in1_size[2], real_T in2,
                         const real_T in3_data[], const int32_T in3_size[2]);

void binary_expand_op_18(real_T in1_data[], int32_T in1_size[2], real_T in2,
                         const real_T in3_data[], const int32_T in3_size[2],
                         const real_T in4_data[], const int32_T in4_size[2]);

void dynamic_size_checks(const emlrtStack *sp, int32_T b_size,
                         int32_T innerDimA, int32_T innerDimB);

/* End of code generation (eml_mtimes_helper.h) */
