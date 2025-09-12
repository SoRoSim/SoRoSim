/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * eml_mtimes_helper.c
 *
 * Code generation for function 'eml_mtimes_helper'
 *
 */

/* Include files */
#include "eml_mtimes_helper.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>

/* Function Definitions */
void binary_expand_op(real_T in1_data[], int32_T in1_size[2], real_T in2,
                      const real_T in3_data[], const int32_T in3_size[2])
{
  real_T in2_data[600];
  int32_T aux_0_1;
  int32_T aux_1_1;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  if (in3_size[1] == 1) {
    loop_ub = in1_size[1];
  } else {
    loop_ub = in3_size[1];
  }
  stride_0_1 = (in1_size[1] != 1);
  stride_1_1 = (in3_size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < loop_ub; i++) {
    __m128d r;
    __m128d r1;
    __m128d r2;
    r = _mm_loadu_pd(&in1_data[6 * aux_0_1]);
    r1 = _mm_loadu_pd(&in3_data[6 * aux_1_1]);
    r2 = _mm_set1_pd(in2);
    _mm_storeu_pd(&in2_data[6 * i], _mm_mul_pd(r2, _mm_sub_pd(r, r1)));
    r = _mm_loadu_pd(&in1_data[6 * aux_0_1 + 2]);
    r1 = _mm_loadu_pd(&in3_data[6 * aux_1_1 + 2]);
    _mm_storeu_pd(&in2_data[6 * i + 2], _mm_mul_pd(r2, _mm_sub_pd(r, r1)));
    r = _mm_loadu_pd(&in1_data[6 * aux_0_1 + 4]);
    r1 = _mm_loadu_pd(&in3_data[6 * aux_1_1 + 4]);
    _mm_storeu_pd(&in2_data[6 * i + 4], _mm_mul_pd(r2, _mm_sub_pd(r, r1)));
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
  in1_size[0] = 6;
  in1_size[1] = loop_ub;
  for (i = 0; i < loop_ub; i++) {
    for (stride_1_1 = 0; stride_1_1 < 6; stride_1_1++) {
      stride_0_1 = stride_1_1 + 6 * i;
      in1_data[stride_0_1] = in2_data[stride_0_1];
    }
  }
}

void binary_expand_op_1(real_T in1_data[], int32_T in1_size[2], real_T in2,
                        const real_T in3_data[], const int32_T in3_size[2],
                        const real_T in4_data[], const int32_T in4_size[2])
{
  int32_T aux_0_1;
  int32_T aux_1_1;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  in1_size[0] = 6;
  if (in4_size[1] == 1) {
    loop_ub = in3_size[1];
  } else {
    loop_ub = in4_size[1];
  }
  in1_size[1] = loop_ub;
  stride_0_1 = (in3_size[1] != 1);
  stride_1_1 = (in4_size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < loop_ub; i++) {
    __m128d r;
    __m128d r1;
    __m128d r2;
    r = _mm_loadu_pd(&in3_data[6 * aux_0_1]);
    r1 = _mm_loadu_pd(&in4_data[6 * aux_1_1]);
    r2 = _mm_set1_pd(in2);
    _mm_storeu_pd(&in1_data[6 * i], _mm_mul_pd(r2, _mm_add_pd(r, r1)));
    r = _mm_loadu_pd(&in3_data[6 * aux_0_1 + 2]);
    r1 = _mm_loadu_pd(&in4_data[6 * aux_1_1 + 2]);
    _mm_storeu_pd(&in1_data[6 * i + 2], _mm_mul_pd(r2, _mm_add_pd(r, r1)));
    r = _mm_loadu_pd(&in3_data[6 * aux_0_1 + 4]);
    r1 = _mm_loadu_pd(&in4_data[6 * aux_1_1 + 4]);
    _mm_storeu_pd(&in1_data[6 * i + 4], _mm_mul_pd(r2, _mm_add_pd(r, r1)));
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
}

/* End of code generation (eml_mtimes_helper.c) */
