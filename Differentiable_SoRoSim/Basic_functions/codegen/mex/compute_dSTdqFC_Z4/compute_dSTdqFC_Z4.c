/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compute_dSTdqFC_Z4.c
 *
 * Code generation for function 'compute_dSTdqFC_Z4'
 *
 */

/* Include files */
#include "compute_dSTdqFC_Z4.h"
#include "compute_dSTdqFC_Z4_data.h"
#include "compute_dSTdqFC_Z4_emxutil.h"
#include "compute_dSTdqFC_Z4_types.h"
#include "dinamico_coadjbar.h"
#include "eml_mtimes_helper.h"
#include "mpower.h"
#include "mtimes.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = {
    3,                    /* lineNo */
    "compute_dSTdqFC_Z4", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z4.m" /* pathName */
};

static emlrtRSInfo b_emlrtRSI = {
    10,                   /* lineNo */
    "compute_dSTdqFC_Z4", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z4.m" /* pathName */
};

static emlrtRSInfo c_emlrtRSI = {
    22,                   /* lineNo */
    "compute_dSTdqFC_Z4", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z4.m" /* pathName */
};

static emlrtRSInfo d_emlrtRSI = {
    30,                   /* lineNo */
    "compute_dSTdqFC_Z4", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z4.m" /* pathName */
};

static emlrtRSInfo e_emlrtRSI = {
    31,                   /* lineNo */
    "compute_dSTdqFC_Z4", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z4.m" /* pathName */
};

static emlrtRSInfo f_emlrtRSI = {
    43,                   /* lineNo */
    "compute_dSTdqFC_Z4", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z4.m" /* pathName */
};

static emlrtRSInfo i_emlrtRSI =
    {
        94,                  /* lineNo */
        "eml_mtimes_helper", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pathName */
};

static emlrtRSInfo l_emlrtRSI =
    {
        69,                  /* lineNo */
        "eml_mtimes_helper", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pathName */
};

static emlrtRTEInfo emlrtRTEI =
    {
        138,                   /* lineNo */
        23,                    /* colNo */
        "dynamic_size_checks", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2024a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pName */
};

static emlrtRTEInfo b_emlrtRTEI =
    {
        133,                   /* lineNo */
        23,                    /* colNo */
        "dynamic_size_checks", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2024a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pName */
};

static emlrtECInfo emlrtECI = {
    2,                    /* nDims */
    43,                   /* lineNo */
    24,                   /* colNo */
    "compute_dSTdqFC_Z4", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z4.m" /* pName */
};

static emlrtECInfo b_emlrtECI = {
    1,                    /* nDims */
    43,                   /* lineNo */
    24,                   /* colNo */
    "compute_dSTdqFC_Z4", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z4.m" /* pName */
};

static emlrtBCInfo emlrtBCI = {
    1,                    /* iFirst */
    24,                   /* iLast */
    41,                   /* lineNo */
    56,                   /* colNo */
    "coadjOmegap",        /* aName */
    "compute_dSTdqFC_Z4", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z4.m", /* pName */
    0       /* checkKind */
};

static emlrtBCInfo b_emlrtBCI = {
    1,                    /* iFirst */
    24,                   /* iLast */
    41,                   /* lineNo */
    44,                   /* colNo */
    "coadjOmegap",        /* aName */
    "compute_dSTdqFC_Z4", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z4.m", /* pName */
    0       /* checkKind */
};

static emlrtECInfo c_emlrtECI = {
    -1,                   /* nDims */
    31,                   /* lineNo */
    9,                    /* colNo */
    "compute_dSTdqFC_Z4", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z4.m" /* pName */
};

static emlrtECInfo d_emlrtECI = {
    2,                    /* nDims */
    30,                   /* lineNo */
    20,                   /* colNo */
    "compute_dSTdqFC_Z4", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z4.m" /* pName */
};

static emlrtECInfo e_emlrtECI = {
    1,                    /* nDims */
    30,                   /* lineNo */
    20,                   /* colNo */
    "compute_dSTdqFC_Z4", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z4.m" /* pName */
};

static emlrtECInfo f_emlrtECI = {
    2,                    /* nDims */
    22,                   /* lineNo */
    24,                   /* colNo */
    "compute_dSTdqFC_Z4", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z4.m" /* pName */
};

static emlrtECInfo g_emlrtECI = {
    1,                    /* nDims */
    22,                   /* lineNo */
    24,                   /* colNo */
    "compute_dSTdqFC_Z4", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z4.m" /* pName */
};

static emlrtBCInfo c_emlrtBCI = {
    1,                    /* iFirst */
    24,                   /* iLast */
    20,                   /* lineNo */
    56,                   /* colNo */
    "coadjOmegap",        /* aName */
    "compute_dSTdqFC_Z4", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z4.m", /* pName */
    0       /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = {
    1,                    /* iFirst */
    24,                   /* iLast */
    20,                   /* lineNo */
    44,                   /* colNo */
    "coadjOmegap",        /* aName */
    "compute_dSTdqFC_Z4", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z4.m", /* pName */
    0       /* checkKind */
};

static emlrtECInfo h_emlrtECI = {
    -1,                   /* nDims */
    10,                   /* lineNo */
    9,                    /* colNo */
    "compute_dSTdqFC_Z4", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z4.m" /* pName */
};

static emlrtECInfo i_emlrtECI = {
    2,                    /* nDims */
    3,                    /* lineNo */
    28,                   /* colNo */
    "compute_dSTdqFC_Z4", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z4.m" /* pName */
};

static emlrtECInfo j_emlrtECI = {
    1,                    /* nDims */
    3,                    /* lineNo */
    28,                   /* colNo */
    "compute_dSTdqFC_Z4", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z4.m" /* pName */
};

static emlrtRTEInfo e_emlrtRTEI = {
    3,                    /* lineNo */
    28,                   /* colNo */
    "compute_dSTdqFC_Z4", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z4.m" /* pName */
};

static emlrtRTEInfo h_emlrtRTEI = {
    22,                   /* lineNo */
    24,                   /* colNo */
    "compute_dSTdqFC_Z4", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z4.m" /* pName */
};

static emlrtRSInfo m_emlrtRSI =
    {
        76,                  /* lineNo */
        "eml_mtimes_helper", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pathName */
};

/* Function Declarations */
static void plus(const emlrtStack *sp, emxArray_real_T *in1,
                 const emxArray_real_T *in2);

/* Function Definitions */
static void plus(const emlrtStack *sp, emxArray_real_T *in1,
                 const emxArray_real_T *in2)
{
  emxArray_real_T *b_in1;
  const real_T *in2_data;
  real_T *b_in1_data;
  real_T *in1_data;
  int32_T aux_0_1;
  int32_T aux_1_1;
  int32_T b_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_0_1;
  int32_T stride_1_0;
  int32_T stride_1_1;
  in2_data = in2->data;
  in1_data = in1->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &b_in1, &h_emlrtRTEI);
  if (in2->size[0] == 1) {
    loop_ub = in1->size[0];
  } else {
    loop_ub = in2->size[0];
  }
  i = b_in1->size[0] * b_in1->size[1];
  b_in1->size[0] = loop_ub;
  if (in2->size[1] == 1) {
    b_loop_ub = in1->size[1];
  } else {
    b_loop_ub = in2->size[1];
  }
  b_in1->size[1] = b_loop_ub;
  emxEnsureCapacity_real_T(sp, b_in1, i, &h_emlrtRTEI);
  b_in1_data = b_in1->data;
  stride_0_0 = (in1->size[0] != 1);
  stride_0_1 = (in1->size[1] != 1);
  stride_1_0 = (in2->size[0] != 1);
  stride_1_1 = (in2->size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < b_loop_ub; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      b_in1_data[i1 + b_in1->size[0] * i] =
          in1_data[i1 * stride_0_0 + in1->size[0] * aux_0_1] +
          in2_data[i1 * stride_1_0 + in2->size[0] * aux_1_1];
    }
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
  i = in1->size[0] * in1->size[1];
  in1->size[0] = loop_ub;
  in1->size[1] = b_loop_ub;
  emxEnsureCapacity_real_T(sp, in1, i, &h_emlrtRTEI);
  in1_data = in1->data;
  for (i = 0; i < b_loop_ub; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      in1_data[i1 + in1->size[0] * i] = b_in1_data[i1 + b_in1->size[0] * i];
    }
  }
  emxFree_real_T(sp, &b_in1);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void compute_dSTdqFC_Z4(const emlrtStack *sp, real_T h, const real_T Omega[6],
                        const real_T Phi_Z1_data[],
                        const int32_T Phi_Z1_size[2],
                        const real_T Phi_Z2_data[],
                        const int32_T Phi_Z2_size[2], const real_T Z_data[],
                        const int32_T Z_size[2], const real_T T[36],
                        const real_T f[4], const real_T fd[4],
                        const real_T adjOmegap[144], const real_T F_C[6],
                        emxArray_real_T *dSTdq_FC)
{
  static const real_T B[36] = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  __m128d r2;
  __m128d r3;
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack st;
  emxArray_real_T *r;
  real_T b_y_data[600];
  real_T y_data[600];
  real_T y_tmp_data[600];
  real_T coadjOmegap[144];
  real_T coadjOmegap1_data[108];
  real_T coadjOmegap2_data[108];
  real_T d_y_data[100];
  real_T b_I[36];
  real_T y_tmp[6];
  real_T alpha1;
  real_T beta1;
  real_T theta;
  real_T *dSTdq_FC_data;
  real_T *r1;
  int32_T adjOmegap_size[2];
  int32_T c_y_size[2];
  int32_T coadjOmegap1_size[2];
  int32_T coadjOmegap2_size[2];
  int32_T iv[2];
  int32_T tmp_size[2];
  int32_T y_size[2];
  int32_T b_r;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T loop_ub;
  int32_T scalarLB;
  int32_T u;
  int32_T vectorUB;
  char_T TRANSA1;
  char_T TRANSB1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  for (i = 0; i < 6; i++) {
    alpha1 = 0.0;
    for (i1 = 0; i1 < 6; i1++) {
      alpha1 += T[i1 + 6 * i] * F_C[i1];
    }
    y_tmp[i] = alpha1;
  }
  st.site = &emlrtRSI;
  b_I[0] = -0.0;
  b_I[6] = y_tmp[2];
  b_I[12] = -y_tmp[1];
  b_I[18] = -0.0;
  b_I[24] = y_tmp[5];
  b_I[30] = -y_tmp[4];
  b_I[1] = -y_tmp[2];
  b_I[7] = -0.0;
  b_I[13] = y_tmp[0];
  b_I[19] = -y_tmp[5];
  b_I[25] = -0.0;
  b_I[31] = y_tmp[3];
  b_I[2] = y_tmp[1];
  b_I[8] = -y_tmp[0];
  b_I[14] = -0.0;
  b_I[20] = y_tmp[4];
  b_I[26] = -y_tmp[3];
  b_I[32] = -0.0;
  b_I[3] = -0.0;
  b_I[9] = y_tmp[5];
  b_I[15] = -y_tmp[4];
  b_I[21] = -0.0;
  b_I[27] = -0.0;
  b_I[33] = -0.0;
  b_I[4] = -y_tmp[5];
  b_I[10] = -0.0;
  b_I[16] = y_tmp[3];
  b_I[22] = -0.0;
  b_I[28] = -0.0;
  b_I[34] = -0.0;
  b_I[5] = y_tmp[4];
  b_I[11] = -y_tmp[3];
  b_I[17] = -0.0;
  b_I[23] = -0.0;
  b_I[29] = -0.0;
  b_I[35] = -0.0;
  mtimes(Phi_Z1_data, Phi_Z1_size, b_I, y_data, y_size);
  st.site = &emlrtRSI;
  b_st.site = &i_emlrtRSI;
  b_mtimes(&b_st, y_data, y_size, Phi_Z2_data, Phi_Z2_size, dSTdq_FC);
  dSTdq_FC_data = dSTdq_FC->data;
  st.site = &emlrtRSI;
  b_I[0] = -0.0;
  b_I[6] = y_tmp[2];
  b_I[12] = -y_tmp[1];
  b_I[18] = -0.0;
  b_I[24] = y_tmp[5];
  b_I[30] = -y_tmp[4];
  b_I[1] = -y_tmp[2];
  b_I[7] = -0.0;
  b_I[13] = y_tmp[0];
  b_I[19] = -y_tmp[5];
  b_I[25] = -0.0;
  b_I[31] = y_tmp[3];
  b_I[2] = y_tmp[1];
  b_I[8] = -y_tmp[0];
  b_I[14] = -0.0;
  b_I[20] = y_tmp[4];
  b_I[26] = -y_tmp[3];
  b_I[32] = -0.0;
  b_I[3] = -0.0;
  b_I[9] = y_tmp[5];
  b_I[15] = -y_tmp[4];
  b_I[21] = -0.0;
  b_I[27] = -0.0;
  b_I[33] = -0.0;
  b_I[4] = -y_tmp[5];
  b_I[10] = -0.0;
  b_I[16] = y_tmp[3];
  b_I[22] = -0.0;
  b_I[28] = -0.0;
  b_I[34] = -0.0;
  b_I[5] = y_tmp[4];
  b_I[11] = -y_tmp[3];
  b_I[17] = -0.0;
  b_I[23] = -0.0;
  b_I[29] = -0.0;
  b_I[35] = -0.0;
  mtimes(Phi_Z2_data, Phi_Z2_size, b_I, y_data, y_size);
  st.site = &emlrtRSI;
  emxInit_real_T(&st, &r, &e_emlrtRTEI);
  b_st.site = &i_emlrtRSI;
  b_mtimes(&b_st, y_data, y_size, Phi_Z1_data, Phi_Z1_size, r);
  r1 = r->data;
  if ((dSTdq_FC->size[0] != r->size[0]) &&
      ((dSTdq_FC->size[0] != 1) && (r->size[0] != 1))) {
    emlrtDimSizeImpxCheckR2021b(dSTdq_FC->size[0], r->size[0], &j_emlrtECI,
                                (emlrtConstCTX)sp);
  }
  if ((dSTdq_FC->size[1] != r->size[1]) &&
      ((dSTdq_FC->size[1] != 1) && (r->size[1] != 1))) {
    emlrtDimSizeImpxCheckR2021b(dSTdq_FC->size[1], r->size[1], &i_emlrtECI,
                                (emlrtConstCTX)sp);
  }
  st.site = &emlrtRSI;
  st.site = &emlrtRSI;
  alpha1 = 0.14433756729740643 * (h * h);
  if ((dSTdq_FC->size[0] == r->size[0]) && (dSTdq_FC->size[1] == r->size[1])) {
    loop_ub = dSTdq_FC->size[0] * dSTdq_FC->size[1];
    scalarLB = (loop_ub / 2) << 1;
    vectorUB = scalarLB - 2;
    for (i = 0; i <= vectorUB; i += 2) {
      r2 = _mm_loadu_pd(&dSTdq_FC_data[i]);
      r3 = _mm_loadu_pd(&r1[i]);
      _mm_storeu_pd(&dSTdq_FC_data[i],
                    _mm_mul_pd(_mm_set1_pd(alpha1), _mm_sub_pd(r2, r3)));
    }
    for (i = scalarLB; i < loop_ub; i++) {
      dSTdq_FC_data[i] = alpha1 * (dSTdq_FC_data[i] - r1[i]);
    }
  } else {
    st.site = &m_emlrtRSI;
    binary_expand_op(&st, dSTdq_FC, alpha1, r);
    dSTdq_FC_data = dSTdq_FC->data;
  }
  theta = b_norm(&Omega[0]);
  memset(&coadjOmegap[0], 0, 144U * sizeof(real_T));
  if (theta <= 0.01) {
    int32_T loop_ub_tmp;
    loop_ub_tmp = Z_size[1];
    tmp_size[0] = 6;
    for (b_r = 0; b_r < 4; b_r++) {
      real_T adjOmegap_data[144];
      i = b_r * 6;
      i1 = 6 * (b_r + 1);
      if (i + 1 > i1) {
        i = 0;
        i1 = 0;
        i2 = 0;
        u = 0;
      } else {
        i2 = i;
        u = i1;
      }
      st.site = &b_emlrtRSI;
      alpha1 = mpower(&st, (real_T)b_r + 1.0);
      loop_ub = i1 - i;
      tmp_size[1] = loop_ub;
      for (i1 = 0; i1 < loop_ub; i1++) {
        for (scalarLB = 0; scalarLB < 6; scalarLB++) {
          adjOmegap_data[scalarLB + 6 * i1] =
              alpha1 * adjOmegap[(i + i1) + 24 * scalarLB];
        }
      }
      loop_ub = u - i2;
      iv[0] = loop_ub;
      iv[1] = 6;
      emlrtSubAssignSizeCheckR2012b(&iv[0], 2, &tmp_size[0], 2, &h_emlrtECI,
                                    (emlrtCTX)sp);
      for (i = 0; i < 6; i++) {
        for (i1 = 0; i1 < loop_ub; i1++) {
          coadjOmegap[(i2 + i1) + 24 * i] = adjOmegap_data[i1 + loop_ub * i];
        }
      }
      /*  good because u is from 1 to r */
      for (u = 0; u <= b_r; u++) {
        if (u + 1 == 1) {
          memset(&b_I[0], 0, 36U * sizeof(real_T));
          for (loop_ub = 0; loop_ub < 6; loop_ub++) {
            b_I[loop_ub + 6 * loop_ub] = 1.0;
          }
          coadjOmegap1_size[0] = 6;
          coadjOmegap1_size[1] = 6;
          memcpy(&coadjOmegap1_data[0], &b_I[0], 36U * sizeof(real_T));
        } else {
          i = (u - 1) * 6;
          i1 = 6 * u;
          if (i + 1 > i1) {
            i = 0;
            i1 = 0;
          }
          loop_ub = i1 - i;
          coadjOmegap1_size[0] = loop_ub;
          coadjOmegap1_size[1] = 6;
          for (i1 = 0; i1 < 6; i1++) {
            for (i2 = 0; i2 < loop_ub; i2++) {
              coadjOmegap1_data[i2 + loop_ub * i1] =
                  coadjOmegap[(i + i2) + 24 * i1];
            }
          }
          /* coadjOmega^(u-1) */
        }
        if (u == b_r) {
          memset(&b_I[0], 0, 36U * sizeof(real_T));
          for (loop_ub = 0; loop_ub < 6; loop_ub++) {
            b_I[loop_ub + 6 * loop_ub] = 1.0;
          }
          coadjOmegap2_size[0] = 6;
          coadjOmegap2_size[1] = 6;
          memcpy(&coadjOmegap2_data[0], &b_I[0], 36U * sizeof(real_T));
        } else {
          i = b_r - u;
          i1 = (i - 1) * 6 + 1;
          i *= 6;
          if (i1 > i) {
            i1 = 0;
            i = 0;
          } else {
            if (i1 < 1) {
              emlrtDynamicBoundsCheckR2012b(i1, 1, 24, &d_emlrtBCI,
                                            (emlrtConstCTX)sp);
            }
            i1--;
            if (i < 1) {
              emlrtDynamicBoundsCheckR2012b(i, 1, 24, &c_emlrtBCI,
                                            (emlrtConstCTX)sp);
            }
          }
          loop_ub = i - i1;
          coadjOmegap2_size[0] = loop_ub;
          coadjOmegap2_size[1] = 6;
          for (i = 0; i < 6; i++) {
            for (i2 = 0; i2 < loop_ub; i2++) {
              coadjOmegap2_data[i2 + loop_ub * i] =
                  coadjOmegap[(i1 + i2) + 24 * i];
            }
          }
          /* coadjOmega^(r-u) */
        }
        st.site = &c_emlrtRSI;
        alpha1 = mpower(&st, (real_T)b_r + 1.0) * f[b_r];
        st.site = &c_emlrtRSI;
        b_st.site = &l_emlrtRSI;
        if (coadjOmegap1_size[0] != 6) {
          emlrtErrorWithMessageIdR2018a(&b_st, &emlrtRTEI, "MATLAB:innerdim",
                                        "MATLAB:innerdim", 0);
        }
        c_y_size[0] = loop_ub_tmp;
        c_y_size[1] = 6;
        for (i = 0; i < 6; i++) {
          for (i1 = 0; i1 < loop_ub_tmp; i1++) {
            b_y_data[i1 + loop_ub_tmp * i] = alpha1 * Z_data[i + 6 * i1];
          }
        }
        real_T e_y_data[18];
        c_mtimes(b_y_data, c_y_size, coadjOmegap1_data, coadjOmegap1_size,
                 y_data, y_size);
        st.site = &c_emlrtRSI;
        loop_ub = d_mtimes(coadjOmegap2_data, coadjOmegap2_size, F_C, e_y_data);
        st.site = &c_emlrtRSI;
        b_st.site = &c_emlrtRSI;
        dinamico_coadjbar(&b_st, e_y_data, loop_ub, b_I);
        e_mtimes(y_data, y_size, b_I, b_y_data, c_y_size);
        st.site = &c_emlrtRSI;
        b_st.site = &i_emlrtRSI;
        b_mtimes(&b_st, b_y_data, c_y_size, Z_data, Z_size, r);
        r1 = r->data;
        if ((dSTdq_FC->size[0] != r->size[0]) &&
            ((dSTdq_FC->size[0] != 1) && (r->size[0] != 1))) {
          emlrtDimSizeImpxCheckR2021b(dSTdq_FC->size[0], r->size[0],
                                      &g_emlrtECI, (emlrtConstCTX)sp);
        }
        if ((dSTdq_FC->size[1] != r->size[1]) &&
            ((dSTdq_FC->size[1] != 1) && (r->size[1] != 1))) {
          emlrtDimSizeImpxCheckR2021b(dSTdq_FC->size[1], r->size[1],
                                      &f_emlrtECI, (emlrtConstCTX)sp);
        }
        if ((dSTdq_FC->size[0] == r->size[0]) &&
            (dSTdq_FC->size[1] == r->size[1])) {
          loop_ub = dSTdq_FC->size[0] * dSTdq_FC->size[1];
          scalarLB = (loop_ub / 2) << 1;
          vectorUB = scalarLB - 2;
          for (i = 0; i <= vectorUB; i += 2) {
            r2 = _mm_loadu_pd(&dSTdq_FC_data[i]);
            r3 = _mm_loadu_pd(&r1[i]);
            _mm_storeu_pd(&dSTdq_FC_data[i], _mm_add_pd(r2, r3));
          }
          for (i = scalarLB; i < loop_ub; i++) {
            dSTdq_FC_data[i] += r1[i];
          }
        } else {
          st.site = &c_emlrtRSI;
          plus(&st, dSTdq_FC, r);
          dSTdq_FC_data = dSTdq_FC->data;
        }
        if (*emlrtBreakCheckR2012bFlagVar != 0) {
          emlrtBreakCheckR2012b((emlrtConstCTX)sp);
        }
      }
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b((emlrtConstCTX)sp);
      }
    }
  } else {
    int32_T b_loop_ub_tmp;
    int32_T loop_ub_tmp;
    int32_T scalarLB_tmp;
    int32_T vectorUB_tmp;
    loop_ub_tmp = Z_size[1];
    b_loop_ub_tmp = loop_ub_tmp * 6;
    scalarLB_tmp = (b_loop_ub_tmp / 2) << 1;
    vectorUB_tmp = scalarLB_tmp - 2;
    adjOmegap_size[1] = 6;
    tmp_size[0] = 6;
    for (b_r = 0; b_r < 4; b_r++) {
      real_T c_y_data[2400];
      real_T adjOmegap_data[144];
      int32_T b_y_size[2];
      i = b_r * 6;
      i1 = 6 * (b_r + 1);
      if (i + 1 > i1) {
        i2 = 0;
        u = 0;
      } else {
        i2 = i;
        u = i1;
      }
      alpha1 = 1.0 / theta * fd[b_r];
      for (scalarLB = 0; scalarLB < 6; scalarLB++) {
        for (loop_ub = 0; loop_ub < loop_ub_tmp; loop_ub++) {
          y_tmp_data[loop_ub + loop_ub_tmp * scalarLB] =
              Z_data[scalarLB + 6 * loop_ub];
        }
      }
      st.site = &d_emlrtRSI;
      y_size[0] = loop_ub_tmp;
      y_size[1] = 6;
      for (scalarLB = 0; scalarLB <= vectorUB_tmp; scalarLB += 2) {
        r2 = _mm_loadu_pd(&y_tmp_data[scalarLB]);
        _mm_storeu_pd(&y_data[scalarLB], _mm_mul_pd(_mm_set1_pd(alpha1), r2));
      }
      for (scalarLB = scalarLB_tmp; scalarLB < b_loop_ub_tmp; scalarLB++) {
        y_data[scalarLB] = alpha1 * y_tmp_data[scalarLB];
      }
      loop_ub = u - i2;
      adjOmegap_size[0] = loop_ub;
      for (u = 0; u < 6; u++) {
        for (scalarLB = 0; scalarLB < loop_ub; scalarLB++) {
          adjOmegap_data[scalarLB + loop_ub * u] =
              adjOmegap[(i2 + scalarLB) + 24 * u];
        }
      }
      f_mtimes(y_data, y_size, adjOmegap_data, adjOmegap_size, c_y_data,
               b_y_size);
      st.site = &d_emlrtRSI;
      b_st.site = &l_emlrtRSI;
      if (b_y_size[1] != 6) {
        if ((b_y_size[0] == 1) && (b_y_size[1] == 1)) {
          emlrtErrorWithMessageIdR2018a(
              &b_st, &b_emlrtRTEI,
              "Coder:toolbox:mtimes_noDynamicScalarExpansion",
              "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
        } else {
          emlrtErrorWithMessageIdR2018a(&b_st, &emlrtRTEI, "MATLAB:innerdim",
                                        "MATLAB:innerdim", 0);
        }
      }
      loop_ub = g_mtimes(c_y_data, b_y_size, F_C, d_y_data);
      scalarLB = (loop_ub / 2) << 1;
      vectorUB = scalarLB - 2;
      for (i2 = 0; i2 < 6; i2++) {
        for (u = 0; u <= vectorUB; u += 2) {
          r2 = _mm_loadu_pd(&d_y_data[u]);
          _mm_storeu_pd(&y_data[u + loop_ub * i2],
                        _mm_mul_pd(r2, _mm_set1_pd(Omega[i2])));
        }
        for (u = scalarLB; u < loop_ub; u++) {
          y_data[u + loop_ub * i2] = d_y_data[u] * Omega[i2];
        }
      }
      st.site = &d_emlrtRSI;
      if (loop_ub == 0) {
        c_y_size[0] = 0;
        c_y_size[1] = 6;
      } else {
        TRANSB1 = 'N';
        TRANSA1 = 'N';
        alpha1 = 1.0;
        beta1 = 0.0;
        m_t = (ptrdiff_t)loop_ub;
        n_t = (ptrdiff_t)6;
        k_t = (ptrdiff_t)6;
        lda_t = (ptrdiff_t)loop_ub;
        ldb_t = (ptrdiff_t)6;
        ldc_t = (ptrdiff_t)loop_ub;
        c_y_size[0] = loop_ub;
        c_y_size[1] = 6;
        dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &y_data[0], &lda_t,
              (real_T *)&B[0], &ldb_t, &beta1, &b_y_data[0], &ldc_t);
      }
      st.site = &d_emlrtRSI;
      b_st.site = &i_emlrtRSI;
      b_mtimes(&b_st, b_y_data, c_y_size, Z_data, Z_size, r);
      r1 = r->data;
      if ((dSTdq_FC->size[0] != r->size[0]) &&
          ((dSTdq_FC->size[0] != 1) && (r->size[0] != 1))) {
        emlrtDimSizeImpxCheckR2021b(dSTdq_FC->size[0], r->size[0], &e_emlrtECI,
                                    (emlrtConstCTX)sp);
      }
      if ((dSTdq_FC->size[1] != r->size[1]) &&
          ((dSTdq_FC->size[1] != 1) && (r->size[1] != 1))) {
        emlrtDimSizeImpxCheckR2021b(dSTdq_FC->size[1], r->size[1], &d_emlrtECI,
                                    (emlrtConstCTX)sp);
      }
      if ((dSTdq_FC->size[0] == r->size[0]) &&
          (dSTdq_FC->size[1] == r->size[1])) {
        loop_ub = dSTdq_FC->size[0] * dSTdq_FC->size[1];
        scalarLB = (loop_ub / 2) << 1;
        vectorUB = scalarLB - 2;
        for (i2 = 0; i2 <= vectorUB; i2 += 2) {
          r2 = _mm_loadu_pd(&dSTdq_FC_data[i2]);
          r3 = _mm_loadu_pd(&r1[i2]);
          _mm_storeu_pd(&dSTdq_FC_data[i2], _mm_add_pd(r2, r3));
        }
        for (i2 = scalarLB; i2 < loop_ub; i2++) {
          dSTdq_FC_data[i2] += r1[i2];
        }
      } else {
        st.site = &d_emlrtRSI;
        plus(&st, dSTdq_FC, r);
        dSTdq_FC_data = dSTdq_FC->data;
      }
      /* (-1)^r*coadjOmegap = adjOmegap  */
      if (i + 1 > i1) {
        i = 0;
        i1 = 0;
        i2 = 0;
        u = 0;
      } else {
        i2 = i;
        u = i1;
      }
      st.site = &e_emlrtRSI;
      alpha1 = mpower(&st, (real_T)b_r + 1.0);
      loop_ub = i1 - i;
      tmp_size[1] = loop_ub;
      for (i1 = 0; i1 < loop_ub; i1++) {
        for (scalarLB = 0; scalarLB < 6; scalarLB++) {
          adjOmegap_data[scalarLB + 6 * i1] =
              alpha1 * adjOmegap[(i + i1) + 24 * scalarLB];
        }
      }
      loop_ub = u - i2;
      iv[0] = loop_ub;
      iv[1] = 6;
      emlrtSubAssignSizeCheckR2012b(&iv[0], 2, &tmp_size[0], 2, &c_emlrtECI,
                                    (emlrtCTX)sp);
      for (i = 0; i < 6; i++) {
        for (i1 = 0; i1 < loop_ub; i1++) {
          coadjOmegap[(i2 + i1) + 24 * i] = adjOmegap_data[i1 + loop_ub * i];
        }
      }
      /*  good because u is from 1 to r */
      for (u = 0; u <= b_r; u++) {
        if (u + 1 == 1) {
          memset(&b_I[0], 0, 36U * sizeof(real_T));
          for (loop_ub = 0; loop_ub < 6; loop_ub++) {
            b_I[loop_ub + 6 * loop_ub] = 1.0;
          }
          coadjOmegap1_size[0] = 6;
          coadjOmegap1_size[1] = 6;
          memcpy(&coadjOmegap1_data[0], &b_I[0], 36U * sizeof(real_T));
        } else {
          i = (u - 1) * 6;
          i1 = 6 * u;
          if (i + 1 > i1) {
            i = 0;
            i1 = 0;
          }
          loop_ub = i1 - i;
          coadjOmegap1_size[0] = loop_ub;
          coadjOmegap1_size[1] = 6;
          for (i1 = 0; i1 < 6; i1++) {
            for (i2 = 0; i2 < loop_ub; i2++) {
              coadjOmegap1_data[i2 + loop_ub * i1] =
                  coadjOmegap[(i + i2) + 24 * i1];
            }
          }
          /* coadjOmega^(u-1) */
        }
        if (u == b_r) {
          memset(&b_I[0], 0, 36U * sizeof(real_T));
          for (loop_ub = 0; loop_ub < 6; loop_ub++) {
            b_I[loop_ub + 6 * loop_ub] = 1.0;
          }
          coadjOmegap2_size[0] = 6;
          coadjOmegap2_size[1] = 6;
          memcpy(&coadjOmegap2_data[0], &b_I[0], 36U * sizeof(real_T));
        } else {
          i = b_r - u;
          i1 = (i - 1) * 6 + 1;
          i *= 6;
          if (i1 > i) {
            i1 = 0;
            i = 0;
          } else {
            if (i1 < 1) {
              emlrtDynamicBoundsCheckR2012b(i1, 1, 24, &b_emlrtBCI,
                                            (emlrtConstCTX)sp);
            }
            i1--;
            if (i < 1) {
              emlrtDynamicBoundsCheckR2012b(i, 1, 24, &emlrtBCI,
                                            (emlrtConstCTX)sp);
            }
          }
          loop_ub = i - i1;
          coadjOmegap2_size[0] = loop_ub;
          coadjOmegap2_size[1] = 6;
          for (i = 0; i < 6; i++) {
            for (i2 = 0; i2 < loop_ub; i2++) {
              coadjOmegap2_data[i2 + loop_ub * i] =
                  coadjOmegap[(i1 + i2) + 24 * i];
            }
          }
          /* coadjOmega^(r-u) */
        }
        st.site = &f_emlrtRSI;
        alpha1 = mpower(&st, (real_T)b_r + 1.0) * f[b_r];
        st.site = &f_emlrtRSI;
        b_st.site = &l_emlrtRSI;
        if (coadjOmegap1_size[0] != 6) {
          emlrtErrorWithMessageIdR2018a(&b_st, &emlrtRTEI, "MATLAB:innerdim",
                                        "MATLAB:innerdim", 0);
        }
        c_y_size[0] = loop_ub_tmp;
        c_y_size[1] = 6;
        for (i = 0; i <= vectorUB_tmp; i += 2) {
          r2 = _mm_loadu_pd(&y_tmp_data[i]);
          _mm_storeu_pd(&b_y_data[i], _mm_mul_pd(_mm_set1_pd(alpha1), r2));
        }
        for (i = scalarLB_tmp; i < b_loop_ub_tmp; i++) {
          b_y_data[i] = alpha1 * y_tmp_data[i];
        }
        real_T e_y_data[18];
        c_mtimes(b_y_data, c_y_size, coadjOmegap1_data, coadjOmegap1_size,
                 y_data, y_size);
        st.site = &f_emlrtRSI;
        loop_ub = d_mtimes(coadjOmegap2_data, coadjOmegap2_size, F_C, e_y_data);
        st.site = &f_emlrtRSI;
        b_st.site = &f_emlrtRSI;
        dinamico_coadjbar(&b_st, e_y_data, loop_ub, b_I);
        e_mtimes(y_data, y_size, b_I, b_y_data, c_y_size);
        st.site = &f_emlrtRSI;
        b_st.site = &i_emlrtRSI;
        b_mtimes(&b_st, b_y_data, c_y_size, Z_data, Z_size, r);
        r1 = r->data;
        if ((dSTdq_FC->size[0] != r->size[0]) &&
            ((dSTdq_FC->size[0] != 1) && (r->size[0] != 1))) {
          emlrtDimSizeImpxCheckR2021b(dSTdq_FC->size[0], r->size[0],
                                      &b_emlrtECI, (emlrtConstCTX)sp);
        }
        if ((dSTdq_FC->size[1] != r->size[1]) &&
            ((dSTdq_FC->size[1] != 1) && (r->size[1] != 1))) {
          emlrtDimSizeImpxCheckR2021b(dSTdq_FC->size[1], r->size[1], &emlrtECI,
                                      (emlrtConstCTX)sp);
        }
        if ((dSTdq_FC->size[0] == r->size[0]) &&
            (dSTdq_FC->size[1] == r->size[1])) {
          loop_ub = dSTdq_FC->size[0] * dSTdq_FC->size[1];
          scalarLB = (loop_ub / 2) << 1;
          vectorUB = scalarLB - 2;
          for (i = 0; i <= vectorUB; i += 2) {
            r2 = _mm_loadu_pd(&dSTdq_FC_data[i]);
            r3 = _mm_loadu_pd(&r1[i]);
            _mm_storeu_pd(&dSTdq_FC_data[i], _mm_add_pd(r2, r3));
          }
          for (i = scalarLB; i < loop_ub; i++) {
            dSTdq_FC_data[i] += r1[i];
          }
        } else {
          st.site = &f_emlrtRSI;
          plus(&st, dSTdq_FC, r);
          dSTdq_FC_data = dSTdq_FC->data;
        }
        if (*emlrtBreakCheckR2012bFlagVar != 0) {
          emlrtBreakCheckR2012b((emlrtConstCTX)sp);
        }
      }
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b((emlrtConstCTX)sp);
      }
    }
  }
  emxFree_real_T(sp, &r);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (compute_dSTdqFC_Z4.c) */
