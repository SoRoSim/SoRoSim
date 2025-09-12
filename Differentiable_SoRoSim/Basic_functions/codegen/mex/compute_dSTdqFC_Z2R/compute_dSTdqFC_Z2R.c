/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compute_dSTdqFC_Z2R.c
 *
 * Code generation for function 'compute_dSTdqFC_Z2R'
 *
 */

/* Include files */
#include "compute_dSTdqFC_Z2R.h"
#include "compute_dSTdqFC_Z2R_data.h"
#include "compute_dSTdqFC_Z2R_emxutil.h"
#include "compute_dSTdqFC_Z2R_types.h"
#include "dinamico_coadjbar.h"
#include "mtimes.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = {
    10,                    /* lineNo */
    "compute_dSTdqFC_Z2R", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z2R.m" /* pathName */
};

static emlrtRSInfo b_emlrtRSI = {
    22,                    /* lineNo */
    "compute_dSTdqFC_Z2R", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z2R.m" /* pathName */
};

static emlrtRSInfo c_emlrtRSI = {
    30,                    /* lineNo */
    "compute_dSTdqFC_Z2R", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z2R.m" /* pathName */
};

static emlrtRSInfo d_emlrtRSI = {
    31,                    /* lineNo */
    "compute_dSTdqFC_Z2R", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z2R.m" /* pathName */
};

static emlrtRSInfo e_emlrtRSI = {
    43,                    /* lineNo */
    "compute_dSTdqFC_Z2R", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z2R.m" /* pathName */
};

static emlrtRSInfo h_emlrtRSI =
    {
        94,                  /* lineNo */
        "eml_mtimes_helper", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pathName */
};

static emlrtRSInfo i_emlrtRSI =
    {
        69,                  /* lineNo */
        "eml_mtimes_helper", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pathName */
};

static emlrtDCInfo emlrtDCI = {
    3,                     /* lineNo */
    1,                     /* colNo */
    "compute_dSTdqFC_Z2R", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z2R.m", /* pName */
    1        /* checkKind */
};

static emlrtDCInfo b_emlrtDCI = {
    3,                     /* lineNo */
    18,                    /* colNo */
    "compute_dSTdqFC_Z2R", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z2R.m", /* pName */
    4        /* checkKind */
};

static emlrtDCInfo c_emlrtDCI = {
    3,                     /* lineNo */
    18,                    /* colNo */
    "compute_dSTdqFC_Z2R", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z2R.m", /* pName */
    1        /* checkKind */
};

static emlrtECInfo emlrtECI = {
    -1,                    /* nDims */
    10,                    /* lineNo */
    9,                     /* colNo */
    "compute_dSTdqFC_Z2R", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z2R.m" /* pName */
};

static emlrtBCInfo emlrtBCI = {
    1,                     /* iFirst */
    24,                    /* iLast */
    20,                    /* lineNo */
    44,                    /* colNo */
    "coadjOmegap",         /* aName */
    "compute_dSTdqFC_Z2R", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z2R.m", /* pName */
    0        /* checkKind */
};

static emlrtBCInfo b_emlrtBCI = {
    1,                     /* iFirst */
    24,                    /* iLast */
    20,                    /* lineNo */
    56,                    /* colNo */
    "coadjOmegap",         /* aName */
    "compute_dSTdqFC_Z2R", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z2R.m", /* pName */
    0        /* checkKind */
};

static emlrtECInfo b_emlrtECI = {
    1,                     /* nDims */
    22,                    /* lineNo */
    24,                    /* colNo */
    "compute_dSTdqFC_Z2R", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z2R.m" /* pName */
};

static emlrtECInfo c_emlrtECI = {
    2,                     /* nDims */
    22,                    /* lineNo */
    24,                    /* colNo */
    "compute_dSTdqFC_Z2R", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z2R.m" /* pName */
};

static emlrtECInfo d_emlrtECI = {
    1,                     /* nDims */
    30,                    /* lineNo */
    20,                    /* colNo */
    "compute_dSTdqFC_Z2R", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z2R.m" /* pName */
};

static emlrtECInfo e_emlrtECI = {
    2,                     /* nDims */
    30,                    /* lineNo */
    20,                    /* colNo */
    "compute_dSTdqFC_Z2R", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z2R.m" /* pName */
};

static emlrtECInfo f_emlrtECI = {
    -1,                    /* nDims */
    31,                    /* lineNo */
    9,                     /* colNo */
    "compute_dSTdqFC_Z2R", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z2R.m" /* pName */
};

static emlrtBCInfo c_emlrtBCI = {
    1,                     /* iFirst */
    24,                    /* iLast */
    41,                    /* lineNo */
    44,                    /* colNo */
    "coadjOmegap",         /* aName */
    "compute_dSTdqFC_Z2R", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z2R.m", /* pName */
    0        /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = {
    1,                     /* iFirst */
    24,                    /* iLast */
    41,                    /* lineNo */
    56,                    /* colNo */
    "coadjOmegap",         /* aName */
    "compute_dSTdqFC_Z2R", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z2R.m", /* pName */
    0        /* checkKind */
};

static emlrtECInfo g_emlrtECI = {
    1,                     /* nDims */
    43,                    /* lineNo */
    24,                    /* colNo */
    "compute_dSTdqFC_Z2R", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z2R.m" /* pName */
};

static emlrtECInfo h_emlrtECI = {
    2,                     /* nDims */
    43,                    /* lineNo */
    24,                    /* colNo */
    "compute_dSTdqFC_Z2R", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z2R.m" /* pName */
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

static emlrtRTEInfo c_emlrtRTEI = {
    3,                     /* lineNo */
    1,                     /* colNo */
    "compute_dSTdqFC_Z2R", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z2R.m" /* pName */
};

static emlrtRTEInfo d_emlrtRTEI = {
    30,                    /* lineNo */
    20,                    /* colNo */
    "compute_dSTdqFC_Z2R", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z2R.m" /* pName */
};

static emlrtRTEInfo h_emlrtRTEI = {
    22,                    /* lineNo */
    24,                    /* colNo */
    "compute_dSTdqFC_Z2R", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\compute_dSTdqFC_"
    "Z2R.m" /* pName */
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

void compute_dSTdqFC_Z2R(const emlrtStack *sp, real_T ndof,
                         const real_T Omega[6], const real_T Z_data[],
                         const int32_T Z_size[2], const real_T f[4],
                         const real_T fd[4], const real_T adjOmegap[144],
                         const real_T F_C[6], emxArray_real_T *dSTdq_FC)
{
  emlrtStack b_st;
  emlrtStack st;
  emxArray_real_T *r;
  real_T b_y_data[600];
  real_T c_y_data[600];
  real_T y_tmp_data[600];
  real_T coadjOmegap[144];
  real_T coadjOmegap1_data[108];
  real_T coadjOmegap2_data[108];
  real_T d_y_data[100];
  real_T b_I[36];
  real_T theta;
  real_T *dSTdq_FC_data;
  real_T *r2;
  int32_T adjOmegap_size[2];
  int32_T b_y_size[2];
  int32_T coadjOmegap1_size[2];
  int32_T coadjOmegap2_size[2];
  int32_T d_y_size[2];
  int32_T iv[2];
  int32_T tmp_size[2];
  int32_T b_r;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T loop_ub;
  int32_T scalarLB;
  int32_T u;
  int32_T y_size;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  if (!(ndof >= 0.0)) {
    emlrtNonNegativeCheckR2012b(ndof, &b_emlrtDCI, (emlrtConstCTX)sp);
  }
  i = (int32_T)muDoubleScalarFloor(ndof);
  if (ndof != i) {
    emlrtIntegerCheckR2012b(ndof, &c_emlrtDCI, (emlrtConstCTX)sp);
  }
  i1 = dSTdq_FC->size[0] * dSTdq_FC->size[1];
  i2 = (int32_T)ndof;
  dSTdq_FC->size[0] = i2;
  dSTdq_FC->size[1] = i2;
  emxEnsureCapacity_real_T(sp, dSTdq_FC, i1, &c_emlrtRTEI);
  dSTdq_FC_data = dSTdq_FC->data;
  if (i2 != i) {
    emlrtIntegerCheckR2012b(ndof, &emlrtDCI, (emlrtConstCTX)sp);
  }
  loop_ub = i2 * i2;
  for (i = 0; i < loop_ub; i++) {
    dSTdq_FC_data[i] = 0.0;
  }
  theta = b_norm(&Omega[0]);
  memset(&coadjOmegap[0], 0, 144U * sizeof(real_T));
  emxInit_real_T(sp, &r, &d_emlrtRTEI);
  if (theta <= 0.01) {
    int32_T loop_ub_tmp;
    loop_ub_tmp = Z_size[1];
    tmp_size[0] = 6;
    for (b_r = 0; b_r < 4; b_r++) {
      real_T adjOmegap_data[144];
      real_T y;
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
      st.site = &emlrtRSI;
      y_size = (int32_T)muDoubleScalarPower(-1.0, (real_T)b_r + 1.0);
      loop_ub = i1 - i;
      tmp_size[1] = loop_ub;
      for (i1 = 0; i1 < loop_ub; i1++) {
        for (scalarLB = 0; scalarLB < 6; scalarLB++) {
          adjOmegap_data[scalarLB + 6 * i1] =
              (real_T)y_size * adjOmegap[(i + i1) + 24 * scalarLB];
        }
      }
      loop_ub = u - i2;
      iv[0] = loop_ub;
      iv[1] = 6;
      emlrtSubAssignSizeCheckR2012b(&iv[0], 2, &tmp_size[0], 2, &emlrtECI,
                                    (emlrtCTX)sp);
      for (i = 0; i < 6; i++) {
        for (i1 = 0; i1 < loop_ub; i1++) {
          coadjOmegap[(i2 + i1) + 24 * i] = adjOmegap_data[i1 + loop_ub * i];
        }
      }
      /*  good because u is from 1 to r */
      y = (real_T)y_size * f[b_r];
      for (u = 0; u <= b_r; u++) {
        if (u + 1 == 1) {
          memset(&b_I[0], 0, 36U * sizeof(real_T));
          for (y_size = 0; y_size < 6; y_size++) {
            b_I[y_size + 6 * y_size] = 1.0;
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
          for (y_size = 0; y_size < 6; y_size++) {
            b_I[y_size + 6 * y_size] = 1.0;
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
              emlrtDynamicBoundsCheckR2012b(i1, 1, 24, &emlrtBCI,
                                            (emlrtConstCTX)sp);
            }
            i1--;
            if (i < 1) {
              emlrtDynamicBoundsCheckR2012b(i, 1, 24, &b_emlrtBCI,
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
        st.site = &b_emlrtRSI;
        st.site = &b_emlrtRSI;
        b_st.site = &i_emlrtRSI;
        if (coadjOmegap1_size[0] != 6) {
          emlrtErrorWithMessageIdR2018a(&b_st, &emlrtRTEI, "MATLAB:innerdim",
                                        "MATLAB:innerdim", 0);
        }
        d_y_size[0] = loop_ub_tmp;
        d_y_size[1] = 6;
        for (i = 0; i < 6; i++) {
          for (i1 = 0; i1 < loop_ub_tmp; i1++) {
            c_y_data[i1 + loop_ub_tmp * i] = y * Z_data[i + 6 * i1];
          }
        }
        real_T e_y_data[18];
        mtimes(c_y_data, d_y_size, coadjOmegap1_data, coadjOmegap1_size,
               b_y_data, b_y_size);
        st.site = &b_emlrtRSI;
        y_size = b_mtimes(coadjOmegap2_data, coadjOmegap2_size, F_C, e_y_data);
        st.site = &b_emlrtRSI;
        b_st.site = &b_emlrtRSI;
        dinamico_coadjbar(&b_st, e_y_data, y_size, b_I);
        c_mtimes(b_y_data, b_y_size, b_I, c_y_data, d_y_size);
        st.site = &b_emlrtRSI;
        b_st.site = &h_emlrtRSI;
        d_mtimes(&b_st, c_y_data, d_y_size, Z_data, Z_size, r);
        r2 = r->data;
        if ((dSTdq_FC->size[0] != r->size[0]) &&
            ((dSTdq_FC->size[0] != 1) && (r->size[0] != 1))) {
          emlrtDimSizeImpxCheckR2021b(dSTdq_FC->size[0], r->size[0],
                                      &b_emlrtECI, (emlrtConstCTX)sp);
        }
        if ((dSTdq_FC->size[1] != r->size[1]) &&
            ((dSTdq_FC->size[1] != 1) && (r->size[1] != 1))) {
          emlrtDimSizeImpxCheckR2021b(dSTdq_FC->size[1], r->size[1],
                                      &c_emlrtECI, (emlrtConstCTX)sp);
        }
        if ((dSTdq_FC->size[0] == r->size[0]) &&
            (dSTdq_FC->size[1] == r->size[1])) {
          int32_T vectorUB;
          loop_ub = dSTdq_FC->size[0] * dSTdq_FC->size[1];
          scalarLB = (loop_ub / 2) << 1;
          vectorUB = scalarLB - 2;
          for (i = 0; i <= vectorUB; i += 2) {
            __m128d r1;
            __m128d r3;
            r1 = _mm_loadu_pd(&dSTdq_FC_data[i]);
            r3 = _mm_loadu_pd(&r2[i]);
            _mm_storeu_pd(&dSTdq_FC_data[i], _mm_add_pd(r1, r3));
          }
          for (i = scalarLB; i < loop_ub; i++) {
            dSTdq_FC_data[i] += r2[i];
          }
        } else {
          st.site = &b_emlrtRSI;
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
      __m128d r1;
      __m128d r3;
      real_T y_data[2400];
      real_T adjOmegap_data[144];
      real_T y;
      int32_T c_y_size[2];
      int32_T vectorUB;
      i = b_r * 6;
      i1 = 6 * (b_r + 1);
      if (i + 1 > i1) {
        i2 = 0;
        u = 0;
      } else {
        i2 = i;
        u = i1;
      }
      y = 1.0 / theta * fd[b_r];
      for (scalarLB = 0; scalarLB < 6; scalarLB++) {
        for (y_size = 0; y_size < loop_ub_tmp; y_size++) {
          y_tmp_data[y_size + loop_ub_tmp * scalarLB] =
              Z_data[scalarLB + 6 * y_size];
        }
      }
      st.site = &c_emlrtRSI;
      b_y_size[0] = loop_ub_tmp;
      b_y_size[1] = 6;
      for (scalarLB = 0; scalarLB <= vectorUB_tmp; scalarLB += 2) {
        r1 = _mm_loadu_pd(&y_tmp_data[scalarLB]);
        _mm_storeu_pd(&b_y_data[scalarLB], _mm_mul_pd(_mm_set1_pd(y), r1));
      }
      for (scalarLB = scalarLB_tmp; scalarLB < b_loop_ub_tmp; scalarLB++) {
        b_y_data[scalarLB] = y * y_tmp_data[scalarLB];
      }
      loop_ub = u - i2;
      adjOmegap_size[0] = loop_ub;
      for (u = 0; u < 6; u++) {
        for (scalarLB = 0; scalarLB < loop_ub; scalarLB++) {
          adjOmegap_data[scalarLB + loop_ub * u] =
              adjOmegap[(i2 + scalarLB) + 24 * u];
        }
      }
      e_mtimes(b_y_data, b_y_size, adjOmegap_data, adjOmegap_size, y_data,
               c_y_size);
      st.site = &c_emlrtRSI;
      b_st.site = &i_emlrtRSI;
      if (c_y_size[1] != 6) {
        if ((c_y_size[0] == 1) && (c_y_size[1] == 1)) {
          emlrtErrorWithMessageIdR2018a(
              &b_st, &b_emlrtRTEI,
              "Coder:toolbox:mtimes_noDynamicScalarExpansion",
              "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
        } else {
          emlrtErrorWithMessageIdR2018a(&b_st, &emlrtRTEI, "MATLAB:innerdim",
                                        "MATLAB:innerdim", 0);
        }
      }
      y_size = f_mtimes(y_data, c_y_size, F_C, d_y_data);
      st.site = &c_emlrtRSI;
      d_y_size[0] = y_size;
      d_y_size[1] = 6;
      scalarLB = (y_size / 2) << 1;
      vectorUB = scalarLB - 2;
      for (i2 = 0; i2 < 6; i2++) {
        for (u = 0; u <= vectorUB; u += 2) {
          r1 = _mm_loadu_pd(&d_y_data[u]);
          _mm_storeu_pd(&c_y_data[u + y_size * i2],
                        _mm_mul_pd(r1, _mm_set1_pd(Omega[i2])));
        }
        for (u = scalarLB; u < y_size; u++) {
          c_y_data[u + y_size * i2] = d_y_data[u] * Omega[i2];
        }
      }
      g_mtimes(c_y_data, d_y_size, b_y_data, b_y_size);
      st.site = &c_emlrtRSI;
      b_st.site = &h_emlrtRSI;
      d_mtimes(&b_st, b_y_data, b_y_size, Z_data, Z_size, r);
      r2 = r->data;
      if ((dSTdq_FC->size[0] != r->size[0]) &&
          ((dSTdq_FC->size[0] != 1) && (r->size[0] != 1))) {
        emlrtDimSizeImpxCheckR2021b(dSTdq_FC->size[0], r->size[0], &d_emlrtECI,
                                    (emlrtConstCTX)sp);
      }
      if ((dSTdq_FC->size[1] != r->size[1]) &&
          ((dSTdq_FC->size[1] != 1) && (r->size[1] != 1))) {
        emlrtDimSizeImpxCheckR2021b(dSTdq_FC->size[1], r->size[1], &e_emlrtECI,
                                    (emlrtConstCTX)sp);
      }
      if ((dSTdq_FC->size[0] == r->size[0]) &&
          (dSTdq_FC->size[1] == r->size[1])) {
        loop_ub = dSTdq_FC->size[0] * dSTdq_FC->size[1];
        scalarLB = (loop_ub / 2) << 1;
        vectorUB = scalarLB - 2;
        for (i2 = 0; i2 <= vectorUB; i2 += 2) {
          r1 = _mm_loadu_pd(&dSTdq_FC_data[i2]);
          r3 = _mm_loadu_pd(&r2[i2]);
          _mm_storeu_pd(&dSTdq_FC_data[i2], _mm_add_pd(r1, r3));
        }
        for (i2 = scalarLB; i2 < loop_ub; i2++) {
          dSTdq_FC_data[i2] += r2[i2];
        }
      } else {
        st.site = &c_emlrtRSI;
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
      st.site = &d_emlrtRSI;
      y_size = (int32_T)muDoubleScalarPower(-1.0, (real_T)b_r + 1.0);
      loop_ub = i1 - i;
      tmp_size[1] = loop_ub;
      for (i1 = 0; i1 < loop_ub; i1++) {
        for (scalarLB = 0; scalarLB < 6; scalarLB++) {
          adjOmegap_data[scalarLB + 6 * i1] =
              (real_T)y_size * adjOmegap[(i + i1) + 24 * scalarLB];
        }
      }
      loop_ub = u - i2;
      iv[0] = loop_ub;
      iv[1] = 6;
      emlrtSubAssignSizeCheckR2012b(&iv[0], 2, &tmp_size[0], 2, &f_emlrtECI,
                                    (emlrtCTX)sp);
      for (i = 0; i < 6; i++) {
        for (i1 = 0; i1 < loop_ub; i1++) {
          coadjOmegap[(i2 + i1) + 24 * i] = adjOmegap_data[i1 + loop_ub * i];
        }
      }
      /*  good because u is from 1 to r */
      y = (real_T)y_size * f[b_r];
      for (u = 0; u <= b_r; u++) {
        if (u + 1 == 1) {
          memset(&b_I[0], 0, 36U * sizeof(real_T));
          for (y_size = 0; y_size < 6; y_size++) {
            b_I[y_size + 6 * y_size] = 1.0;
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
          for (y_size = 0; y_size < 6; y_size++) {
            b_I[y_size + 6 * y_size] = 1.0;
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
              emlrtDynamicBoundsCheckR2012b(i1, 1, 24, &c_emlrtBCI,
                                            (emlrtConstCTX)sp);
            }
            i1--;
            if (i < 1) {
              emlrtDynamicBoundsCheckR2012b(i, 1, 24, &d_emlrtBCI,
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
        st.site = &e_emlrtRSI;
        st.site = &e_emlrtRSI;
        b_st.site = &i_emlrtRSI;
        if (coadjOmegap1_size[0] != 6) {
          emlrtErrorWithMessageIdR2018a(&b_st, &emlrtRTEI, "MATLAB:innerdim",
                                        "MATLAB:innerdim", 0);
        }
        d_y_size[0] = loop_ub_tmp;
        d_y_size[1] = 6;
        for (i = 0; i <= vectorUB_tmp; i += 2) {
          r1 = _mm_loadu_pd(&y_tmp_data[i]);
          _mm_storeu_pd(&c_y_data[i], _mm_mul_pd(_mm_set1_pd(y), r1));
        }
        for (i = scalarLB_tmp; i < b_loop_ub_tmp; i++) {
          c_y_data[i] = y * y_tmp_data[i];
        }
        real_T e_y_data[18];
        mtimes(c_y_data, d_y_size, coadjOmegap1_data, coadjOmegap1_size,
               b_y_data, b_y_size);
        st.site = &e_emlrtRSI;
        y_size = b_mtimes(coadjOmegap2_data, coadjOmegap2_size, F_C, e_y_data);
        st.site = &e_emlrtRSI;
        b_st.site = &e_emlrtRSI;
        dinamico_coadjbar(&b_st, e_y_data, y_size, b_I);
        c_mtimes(b_y_data, b_y_size, b_I, c_y_data, d_y_size);
        st.site = &e_emlrtRSI;
        b_st.site = &h_emlrtRSI;
        d_mtimes(&b_st, c_y_data, d_y_size, Z_data, Z_size, r);
        r2 = r->data;
        if ((dSTdq_FC->size[0] != r->size[0]) &&
            ((dSTdq_FC->size[0] != 1) && (r->size[0] != 1))) {
          emlrtDimSizeImpxCheckR2021b(dSTdq_FC->size[0], r->size[0],
                                      &g_emlrtECI, (emlrtConstCTX)sp);
        }
        if ((dSTdq_FC->size[1] != r->size[1]) &&
            ((dSTdq_FC->size[1] != 1) && (r->size[1] != 1))) {
          emlrtDimSizeImpxCheckR2021b(dSTdq_FC->size[1], r->size[1],
                                      &h_emlrtECI, (emlrtConstCTX)sp);
        }
        if ((dSTdq_FC->size[0] == r->size[0]) &&
            (dSTdq_FC->size[1] == r->size[1])) {
          loop_ub = dSTdq_FC->size[0] * dSTdq_FC->size[1];
          scalarLB = (loop_ub / 2) << 1;
          vectorUB = scalarLB - 2;
          for (i = 0; i <= vectorUB; i += 2) {
            r1 = _mm_loadu_pd(&dSTdq_FC_data[i]);
            r3 = _mm_loadu_pd(&r2[i]);
            _mm_storeu_pd(&dSTdq_FC_data[i], _mm_add_pd(r1, r3));
          }
          for (i = scalarLB; i < loop_ub; i++) {
            dSTdq_FC_data[i] += r2[i];
          }
        } else {
          st.site = &e_emlrtRSI;
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

/* End of code generation (compute_dSTdqFC_Z2R.c) */
