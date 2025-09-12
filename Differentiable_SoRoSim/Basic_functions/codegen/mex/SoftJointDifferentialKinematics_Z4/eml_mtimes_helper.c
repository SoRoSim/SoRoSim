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
#include "SoftJointDifferentialKinematics_Z4_data.h"
#include "mtimes.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>

/* Variable Definitions */
static emlrtRTEInfo b_emlrtRTEI =
    {
        133,                   /* lineNo */
        23,                    /* colNo */
        "dynamic_size_checks", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2024a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pName */
};

/* Function Definitions */
void binary_expand_op_13(real_T in1_data[], int32_T in1_size[2],
                         const real_T in2_data[], const int32_T *in2_size,
                         const real_T in3_data[], const int32_T in3_size[2],
                         const real_T in4_data[], const int32_T in4_size[2])
{
  real_T b_in3_data[100];
  int32_T b_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  if (in4_size[1] == 1) {
    loop_ub = in3_size[1];
  } else {
    loop_ub = in4_size[1];
  }
  stride_0_1 = (in3_size[1] != 1);
  stride_1_1 = (in4_size[1] != 1);
  b_loop_ub = *in2_size;
  in1_size[0] = b_loop_ub;
  in1_size[1] = loop_ub;
  for (i = 0; i < loop_ub; i++) {
    int32_T scalarLB;
    int32_T vectorUB;
    b_in3_data[i] = in3_data[i * stride_0_1] + in4_data[i * stride_1_1];
    scalarLB = (b_loop_ub / 2) << 1;
    vectorUB = scalarLB - 2;
    for (i1 = 0; i1 <= vectorUB; i1 += 2) {
      __m128d r;
      r = _mm_loadu_pd(&in2_data[i1]);
      _mm_storeu_pd(&in1_data[i1 + in1_size[0] * i],
                    _mm_mul_pd(r, _mm_set1_pd(b_in3_data[i])));
    }
    for (i1 = scalarLB; i1 < b_loop_ub; i1++) {
      in1_data[i1 + in1_size[0] * i] = in2_data[i1] * b_in3_data[i];
    }
  }
}

int32_T binary_expand_op_14(real_T in1_data[], const emlrtRSInfo in2,
                            real_T in3, const real_T in4_data[],
                            const int32_T in4_size[2], const real_T in5_data[],
                            const int32_T in5_size[2], const real_T in6[6])
{
  real_T in3_data[144];
  int32_T in3_size[2];
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  (void)in2;
  if (in5_size[0] == 1) {
    loop_ub = in4_size[0];
  } else {
    loop_ub = in5_size[0];
  }
  in3_size[0] = loop_ub;
  in3_size[1] = 6;
  stride_0_0 = (in4_size[0] != 1);
  stride_1_0 = (in5_size[0] != 1);
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      in3_data[i1 + loop_ub * i] =
          in3 * (in4_data[i1 * stride_0_0 + in4_size[0] * i] +
                 in5_data[i1 * stride_1_0 + in5_size[0] * i]);
    }
  }
  return c_mtimes(in3_data, in3_size, in6, in1_data);
}

void binary_expand_op_15(real_T in1_data[], const emlrtRSInfo in2,
                         const real_T in3[36], real_T in4,
                         const real_T in5_data[], const int32_T in5_size[2],
                         const real_T in6_data[], const int32_T in6_size[2],
                         int32_T in1_size[2])
{
  real_T in4_data[600];
  int32_T in4_size[2];
  int32_T aux_0_1;
  int32_T aux_1_1;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  (void)in2;
  in4_size[0] = 6;
  if (in6_size[1] == 1) {
    loop_ub = in5_size[1];
  } else {
    loop_ub = in6_size[1];
  }
  in4_size[1] = loop_ub;
  stride_0_1 = (in5_size[1] != 1);
  stride_1_1 = (in6_size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < loop_ub; i++) {
    __m128d r;
    __m128d r1;
    __m128d r2;
    r = _mm_loadu_pd(&in5_data[6 * aux_0_1]);
    r1 = _mm_loadu_pd(&in6_data[6 * aux_1_1]);
    r2 = _mm_set1_pd(in4);
    _mm_storeu_pd(&in4_data[6 * i], _mm_mul_pd(r2, _mm_sub_pd(r, r1)));
    r = _mm_loadu_pd(&in5_data[6 * aux_0_1 + 2]);
    r1 = _mm_loadu_pd(&in6_data[6 * aux_1_1 + 2]);
    _mm_storeu_pd(&in4_data[6 * i + 2], _mm_mul_pd(r2, _mm_sub_pd(r, r1)));
    r = _mm_loadu_pd(&in5_data[6 * aux_0_1 + 4]);
    r1 = _mm_loadu_pd(&in6_data[6 * aux_1_1 + 4]);
    _mm_storeu_pd(&in4_data[6 * i + 4], _mm_mul_pd(r2, _mm_sub_pd(r, r1)));
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
  b_mtimes(in3, in4_data, in4_size, in1_data, in1_size);
}

void binary_expand_op_16(real_T in1_data[], int32_T in1_size[2], real_T in2,
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

void binary_expand_op_18(real_T in1_data[], int32_T in1_size[2], real_T in2,
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

void dynamic_size_checks(const emlrtStack *sp, int32_T b_size,
                         int32_T innerDimA, int32_T innerDimB)
{
  if (innerDimA != innerDimB) {
    if (b_size == 1) {
      emlrtErrorWithMessageIdR2018a(
          sp, &b_emlrtRTEI, "Coder:toolbox:mtimes_noDynamicScalarExpansion",
          "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(sp, &emlrtRTEI, "MATLAB:innerdim",
                                    "MATLAB:innerdim", 0);
    }
  }
}

/* End of code generation (eml_mtimes_helper.c) */
