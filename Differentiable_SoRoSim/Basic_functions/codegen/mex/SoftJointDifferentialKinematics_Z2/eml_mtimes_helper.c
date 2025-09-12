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
#include "SoftJointDifferentialKinematics_Z2_data.h"
#include "mtimes.h"
#include "rt_nonfinite.h"

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
int32_T binary_expand_op_9(real_T in1_data[], const emlrtRSInfo in2, real_T in3,
                           const real_T in4_data[], const int32_T in4_size[2],
                           const real_T in5_data[], const int32_T in5_size[2],
                           const real_T in6[6])
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
