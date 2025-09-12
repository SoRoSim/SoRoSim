/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * SoftJointKinematics_Z4.c
 *
 * Code generation for function 'SoftJointKinematics_Z4'
 *
 */

/* Include files */
#include "SoftJointKinematics_Z4.h"
#include "eml_mtimes_helper.h"
#include "mtimes.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = {
    13,                       /* lineNo */
    "SoftJointKinematics_Z4", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\SoftJointKinematics_Z4.m" /* pathName */
};

static emlrtRSInfo b_emlrtRSI = {
    14,                       /* lineNo */
    "SoftJointKinematics_Z4", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\SoftJointKinematics_Z4.m" /* pathName */
};

static emlrtRSInfo c_emlrtRSI = {
    18,                       /* lineNo */
    "SoftJointKinematics_Z4", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\SoftJointKinematics_Z4.m" /* pathName */
};

static emlrtRSInfo d_emlrtRSI = {
    89,                       /* lineNo */
    "SoftJointKinematics_Z4", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\SoftJointKinematics_Z4.m" /* pathName */
};

static emlrtRSInfo f_emlrtRSI =
    {
        69,                  /* lineNo */
        "eml_mtimes_helper", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pathName */
};

static emlrtECInfo emlrtECI = {
    2,                        /* nDims */
    18,                       /* lineNo */
    20,                       /* colNo */
    "SoftJointKinematics_Z4", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\SoftJointKinematics_Z4.m" /* pName */
};

static emlrtECInfo b_emlrtECI = {
    2,                        /* nDims */
    18,                       /* lineNo */
    42,                       /* colNo */
    "SoftJointKinematics_Z4", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\SoftJointKinematics_Z4.m" /* pName */
};

static emlrtECInfo c_emlrtECI = {
    2,                        /* nDims */
    18,                       /* lineNo */
    15,                       /* colNo */
    "SoftJointKinematics_Z4", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\SoftJointKinematics_Z4.m" /* pName */
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

/* Function Declarations */
static void plus(real_T in1_data[], int32_T in1_size[2],
                 const real_T in2_data[], const int32_T in2_size[2]);

/* Function Definitions */
static void plus(real_T in1_data[], int32_T in1_size[2],
                 const real_T in2_data[], const int32_T in2_size[2])
{
  real_T b_in1_data[600];
  int32_T aux_0_1;
  int32_T aux_1_1;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  if (in2_size[1] == 1) {
    loop_ub = in1_size[1];
  } else {
    loop_ub = in2_size[1];
  }
  stride_0_1 = (in1_size[1] != 1);
  stride_1_1 = (in2_size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < loop_ub; i++) {
    __m128d r;
    __m128d r1;
    r = _mm_loadu_pd(&in1_data[6 * aux_0_1]);
    r1 = _mm_loadu_pd(&in2_data[6 * aux_1_1]);
    _mm_storeu_pd(&b_in1_data[6 * i], _mm_add_pd(r, r1));
    r = _mm_loadu_pd(&in1_data[6 * aux_0_1 + 2]);
    r1 = _mm_loadu_pd(&in2_data[6 * aux_1_1 + 2]);
    _mm_storeu_pd(&b_in1_data[6 * i + 2], _mm_add_pd(r, r1));
    r = _mm_loadu_pd(&in1_data[6 * aux_0_1 + 4]);
    r1 = _mm_loadu_pd(&in2_data[6 * aux_1_1 + 4]);
    _mm_storeu_pd(&b_in1_data[6 * i + 4], _mm_add_pd(r, r1));
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
  in1_size[0] = 6;
  in1_size[1] = loop_ub;
  for (i = 0; i < loop_ub; i++) {
    for (stride_1_1 = 0; stride_1_1 < 6; stride_1_1++) {
      stride_0_1 = stride_1_1 + 6 * i;
      in1_data[stride_0_1] = b_in1_data[stride_0_1];
    }
  }
}

void SoftJointKinematics_Z4(
    const emlrtStack *sp, real_T h, const real_T Phi_Z1_data[],
    const int32_T Phi_Z1_size[2], const real_T Phi_Z2_data[],
    const int32_T Phi_Z2_size[2], const real_T xi_star_Z1[6],
    const real_T xi_star_Z2[6], const real_T q_data[], const int32_T q_size[1],
    real_T Omega[6], real_T Z_data[], int32_T Z_size[2], real_T g[16],
    real_T T[36], real_T S_data[], int32_T S_size[2], real_T f[4], real_T fd[4],
    real_T adjOmegap[144])
{
  static const int8_T iv1[36] = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                                 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
  static const int8_T iv[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  __m128d r;
  __m128d r1;
  emlrtStack b_st;
  emlrtStack st;
  real_T b_tmp_data[600];
  real_T tmp_data[600];
  real_T b_tmp[36];
  real_T Omegahatp2[16];
  real_T Omegahatp3[16];
  real_T xi_Z1[6];
  real_T xi_Z2[6];
  real_T Zchp2;
  real_T absxk;
  real_T costheta;
  real_T t;
  real_T theta;
  int32_T b_tmp_size[2];
  int32_T tmp_size[2];
  int32_T b_i;
  int32_T i;
  int32_T scalarLB;
  int32_T vectorUB;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  /* Phi is Phi_scale*Phi_Leg. New Scaleing method! */
  Zchp2 = 0.14433756729740643 * (h * h);
  if (q_size[0] == 0) {
    for (i = 0; i < 6; i++) {
      xi_Z1[i] = xi_star_Z1[i];
      xi_Z2[i] = xi_star_Z2[i];
    }
  } else {
    st.site = &emlrtRSI;
    b_st.site = &f_emlrtRSI;
    if (q_size[0] != Phi_Z1_size[1]) {
      if (q_size[0] == 1) {
        emlrtErrorWithMessageIdR2018a(
            &b_st, &b_emlrtRTEI,
            "Coder:toolbox:mtimes_noDynamicScalarExpansion",
            "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
      } else {
        emlrtErrorWithMessageIdR2018a(&b_st, &emlrtRTEI, "MATLAB:innerdim",
                                      "MATLAB:innerdim", 0);
      }
    }
    mtimes(Phi_Z1_data, Phi_Z1_size, q_data, q_size[0], xi_Z1);
    r = _mm_loadu_pd(&xi_Z1[0]);
    _mm_storeu_pd(&xi_Z1[0], _mm_add_pd(r, _mm_loadu_pd(&xi_star_Z1[0])));
    r = _mm_loadu_pd(&xi_Z1[2]);
    _mm_storeu_pd(&xi_Z1[2], _mm_add_pd(r, _mm_loadu_pd(&xi_star_Z1[2])));
    r = _mm_loadu_pd(&xi_Z1[4]);
    _mm_storeu_pd(&xi_Z1[4], _mm_add_pd(r, _mm_loadu_pd(&xi_star_Z1[4])));
    st.site = &b_emlrtRSI;
    b_st.site = &f_emlrtRSI;
    if (q_size[0] != Phi_Z2_size[1]) {
      if (q_size[0] == 1) {
        emlrtErrorWithMessageIdR2018a(
            &b_st, &b_emlrtRTEI,
            "Coder:toolbox:mtimes_noDynamicScalarExpansion",
            "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
      } else {
        emlrtErrorWithMessageIdR2018a(&b_st, &emlrtRTEI, "MATLAB:innerdim",
                                      "MATLAB:innerdim", 0);
      }
    }
    mtimes(Phi_Z2_data, Phi_Z2_size, q_data, q_size[0], xi_Z2);
    r = _mm_loadu_pd(&xi_Z2[0]);
    _mm_storeu_pd(&xi_Z2[0], _mm_add_pd(r, _mm_loadu_pd(&xi_star_Z2[0])));
    r = _mm_loadu_pd(&xi_Z2[2]);
    _mm_storeu_pd(&xi_Z2[2], _mm_add_pd(r, _mm_loadu_pd(&xi_star_Z2[2])));
    r = _mm_loadu_pd(&xi_Z2[4]);
    _mm_storeu_pd(&xi_Z2[4], _mm_add_pd(r, _mm_loadu_pd(&xi_star_Z2[4])));
  }
  absxk = h / 2.0;
  /*  optimized on 30.05.2022 */
  b_tmp[0] = 0.0;
  b_tmp[6] = -xi_Z1[2];
  b_tmp[12] = xi_Z1[1];
  b_tmp[18] = 0.0;
  b_tmp[24] = 0.0;
  b_tmp[30] = 0.0;
  b_tmp[1] = xi_Z1[2];
  b_tmp[7] = 0.0;
  b_tmp[13] = -xi_Z1[0];
  b_tmp[19] = 0.0;
  b_tmp[25] = 0.0;
  b_tmp[31] = 0.0;
  b_tmp[2] = -xi_Z1[1];
  b_tmp[8] = xi_Z1[0];
  b_tmp[14] = 0.0;
  b_tmp[20] = 0.0;
  b_tmp[26] = 0.0;
  b_tmp[32] = 0.0;
  b_tmp[3] = 0.0;
  b_tmp[9] = -xi_Z1[5];
  b_tmp[15] = xi_Z1[4];
  b_tmp[21] = 0.0;
  b_tmp[27] = -xi_Z1[2];
  b_tmp[33] = xi_Z1[1];
  b_tmp[4] = xi_Z1[5];
  b_tmp[10] = 0.0;
  b_tmp[16] = -xi_Z1[3];
  b_tmp[22] = xi_Z1[2];
  b_tmp[28] = 0.0;
  b_tmp[34] = -xi_Z1[0];
  b_tmp[5] = -xi_Z1[4];
  b_tmp[11] = xi_Z1[3];
  b_tmp[17] = 0.0;
  b_tmp[23] = -xi_Z1[1];
  b_tmp[29] = xi_Z1[0];
  b_tmp[35] = 0.0;
  for (b_i = 0; b_i < 6; b_i++) {
    costheta = 0.0;
    for (vectorUB = 0; vectorUB < 6; vectorUB++) {
      costheta += Zchp2 * b_tmp[b_i + 6 * vectorUB] * xi_Z2[vectorUB];
    }
    Omega[b_i] = absxk * (xi_Z1[b_i] + xi_Z2[b_i]) + costheta;
  }
  if ((Phi_Z1_size[1] != Phi_Z2_size[1]) &&
      ((Phi_Z1_size[1] != 1) && (Phi_Z2_size[1] != 1))) {
    emlrtDimSizeImpxCheckR2021b(Phi_Z1_size[1], Phi_Z2_size[1], &emlrtECI,
                                (emlrtConstCTX)sp);
  }
  st.site = &c_emlrtRSI;
  /*  optimized on 30.05.2022 */
  b_mtimes(b_tmp, Phi_Z2_data, Phi_Z2_size, tmp_data, tmp_size);
  st.site = &c_emlrtRSI;
  /*  optimized on 30.05.2022 */
  b_tmp[0] = 0.0;
  b_tmp[6] = -xi_Z2[2];
  b_tmp[12] = xi_Z2[1];
  b_tmp[18] = 0.0;
  b_tmp[24] = 0.0;
  b_tmp[30] = 0.0;
  b_tmp[1] = xi_Z2[2];
  b_tmp[7] = 0.0;
  b_tmp[13] = -xi_Z2[0];
  b_tmp[19] = 0.0;
  b_tmp[25] = 0.0;
  b_tmp[31] = 0.0;
  b_tmp[2] = -xi_Z2[1];
  b_tmp[8] = xi_Z2[0];
  b_tmp[14] = 0.0;
  b_tmp[20] = 0.0;
  b_tmp[26] = 0.0;
  b_tmp[32] = 0.0;
  b_tmp[3] = 0.0;
  b_tmp[9] = -xi_Z2[5];
  b_tmp[15] = xi_Z2[4];
  b_tmp[21] = 0.0;
  b_tmp[27] = -xi_Z2[2];
  b_tmp[33] = xi_Z2[1];
  b_tmp[4] = xi_Z2[5];
  b_tmp[10] = 0.0;
  b_tmp[16] = -xi_Z2[3];
  b_tmp[22] = xi_Z2[2];
  b_tmp[28] = 0.0;
  b_tmp[34] = -xi_Z2[0];
  b_tmp[5] = -xi_Z2[4];
  b_tmp[11] = xi_Z2[3];
  b_tmp[17] = 0.0;
  b_tmp[23] = -xi_Z2[1];
  b_tmp[29] = xi_Z2[0];
  b_tmp[35] = 0.0;
  b_mtimes(b_tmp, Phi_Z1_data, Phi_Z1_size, b_tmp_data, b_tmp_size);
  if ((tmp_size[1] != b_tmp_size[1]) &&
      ((tmp_size[1] != 1) && (b_tmp_size[1] != 1))) {
    emlrtDimSizeImpxCheckR2021b(tmp_size[1], b_tmp_size[1], &b_emlrtECI,
                                (emlrtConstCTX)sp);
  }
  if (Phi_Z1_size[1] == Phi_Z2_size[1]) {
    Z_size[0] = 6;
    Z_size[1] = Phi_Z1_size[1];
    i = 6 * Phi_Z1_size[1];
    scalarLB = (i / 2) << 1;
    vectorUB = scalarLB - 2;
    for (b_i = 0; b_i <= vectorUB; b_i += 2) {
      _mm_storeu_pd(&Z_data[b_i],
                    _mm_mul_pd(_mm_set1_pd(absxk),
                               _mm_add_pd(_mm_loadu_pd(&Phi_Z1_data[b_i]),
                                          _mm_loadu_pd(&Phi_Z2_data[b_i]))));
    }
    for (b_i = scalarLB; b_i < i; b_i++) {
      Z_data[b_i] = absxk * (Phi_Z1_data[b_i] + Phi_Z2_data[b_i]);
    }
  } else {
    binary_expand_op_1(Z_data, Z_size, absxk, Phi_Z1_data, Phi_Z1_size,
                       Phi_Z2_data, Phi_Z2_size);
  }
  if (tmp_size[1] == b_tmp_size[1]) {
    i = 6 * tmp_size[1];
    tmp_size[0] = 6;
    scalarLB = (i / 2) << 1;
    vectorUB = scalarLB - 2;
    for (b_i = 0; b_i <= vectorUB; b_i += 2) {
      r = _mm_loadu_pd(&tmp_data[b_i]);
      r1 = _mm_loadu_pd(&b_tmp_data[b_i]);
      _mm_storeu_pd(&tmp_data[b_i],
                    _mm_mul_pd(_mm_set1_pd(Zchp2), _mm_sub_pd(r, r1)));
    }
    for (b_i = scalarLB; b_i < i; b_i++) {
      tmp_data[b_i] = Zchp2 * (tmp_data[b_i] - b_tmp_data[b_i]);
    }
  } else {
    binary_expand_op(tmp_data, tmp_size, Zchp2, b_tmp_data, b_tmp_size);
  }
  if ((Z_size[1] != tmp_size[1]) && ((Z_size[1] != 1) && (tmp_size[1] != 1))) {
    emlrtDimSizeImpxCheckR2021b(Z_size[1], tmp_size[1], &c_emlrtECI,
                                (emlrtConstCTX)sp);
  }
  if (Z_size[1] == tmp_size[1]) {
    i = 6 * Z_size[1];
    Z_size[0] = 6;
    scalarLB = (i / 2) << 1;
    vectorUB = scalarLB - 2;
    for (b_i = 0; b_i <= vectorUB; b_i += 2) {
      r = _mm_loadu_pd(&Z_data[b_i]);
      r1 = _mm_loadu_pd(&tmp_data[b_i]);
      _mm_storeu_pd(&Z_data[b_i], _mm_add_pd(r, r1));
    }
    for (b_i = scalarLB; b_i < i; b_i++) {
      Z_data[b_i] += tmp_data[b_i];
    }
  } else {
    plus(Z_data, Z_size, tmp_data, tmp_size);
  }
  /*     %% defining T, Td, f, fd, fdd, adjOmegap (powers of adjOmega) */
  fd[0] = 0.0;
  fd[1] = 0.0;
  fd[2] = 0.0;
  fd[3] = 0.0;
  memset(&adjOmegap[0], 0, 144U * sizeof(real_T));
  /* all powers of adjGamma || INCLUDE POWER OF 0 to avoid an if */
  Zchp2 = 3.3121686421112381E-170;
  absxk = muDoubleScalarAbs(Omega[0]);
  if (absxk > 3.3121686421112381E-170) {
    theta = 1.0;
    Zchp2 = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    theta = t * t;
  }
  absxk = muDoubleScalarAbs(Omega[1]);
  if (absxk > Zchp2) {
    t = Zchp2 / absxk;
    theta = theta * t * t + 1.0;
    Zchp2 = absxk;
  } else {
    t = absxk / Zchp2;
    theta += t * t;
  }
  absxk = muDoubleScalarAbs(Omega[2]);
  if (absxk > Zchp2) {
    t = Zchp2 / absxk;
    theta = theta * t * t + 1.0;
    Zchp2 = absxk;
  } else {
    t = absxk / Zchp2;
    theta += t * t;
  }
  theta = Zchp2 * muDoubleScalarSqrt(theta);
  /*  optimized on 30.05.2022 */
  g[0] = 0.0;
  g[4] = -Omega[2];
  g[8] = Omega[1];
  g[12] = Omega[3];
  g[1] = Omega[2];
  g[5] = 0.0;
  g[9] = -Omega[0];
  g[13] = Omega[4];
  g[2] = -Omega[1];
  g[6] = Omega[0];
  g[10] = 0.0;
  g[14] = Omega[5];
  g[3] = 0.0;
  g[7] = 0.0;
  g[11] = 0.0;
  g[15] = 0.0;
  for (b_i = 0; b_i < 4; b_i++) {
    for (vectorUB = 0; vectorUB < 4; vectorUB++) {
      scalarLB = vectorUB << 2;
      Omegahatp2[b_i + scalarLB] =
          ((g[b_i] * g[scalarLB] + g[b_i + 4] * g[scalarLB + 1]) +
           g[b_i + 8] * g[scalarLB + 2]) +
          g[b_i + 12] * g[scalarLB + 3];
    }
    costheta = Omegahatp2[b_i];
    Zchp2 = Omegahatp2[b_i + 4];
    absxk = Omegahatp2[b_i + 8];
    t = Omegahatp2[b_i + 12];
    for (vectorUB = 0; vectorUB < 4; vectorUB++) {
      scalarLB = vectorUB << 2;
      Omegahatp3[b_i + scalarLB] =
          ((costheta * g[scalarLB] + Zchp2 * g[scalarLB + 1]) +
           absxk * g[scalarLB + 2]) +
          t * g[scalarLB + 3];
    }
  }
  /*  optimized on 30.05.2022 */
  adjOmegap[0] = 0.0;
  adjOmegap[24] = -Omega[2];
  adjOmegap[48] = Omega[1];
  adjOmegap[72] = 0.0;
  adjOmegap[96] = 0.0;
  adjOmegap[120] = 0.0;
  adjOmegap[1] = Omega[2];
  adjOmegap[25] = 0.0;
  adjOmegap[49] = -Omega[0];
  adjOmegap[73] = 0.0;
  adjOmegap[97] = 0.0;
  adjOmegap[121] = 0.0;
  adjOmegap[2] = -Omega[1];
  adjOmegap[26] = Omega[0];
  adjOmegap[50] = 0.0;
  adjOmegap[74] = 0.0;
  adjOmegap[98] = 0.0;
  adjOmegap[122] = 0.0;
  adjOmegap[3] = 0.0;
  adjOmegap[27] = -Omega[5];
  adjOmegap[51] = Omega[4];
  adjOmegap[75] = 0.0;
  adjOmegap[99] = -Omega[2];
  adjOmegap[123] = Omega[1];
  adjOmegap[4] = Omega[5];
  adjOmegap[28] = 0.0;
  adjOmegap[52] = -Omega[3];
  adjOmegap[76] = Omega[2];
  adjOmegap[100] = 0.0;
  adjOmegap[124] = -Omega[0];
  adjOmegap[5] = -Omega[4];
  adjOmegap[29] = Omega[3];
  adjOmegap[53] = 0.0;
  adjOmegap[77] = -Omega[1];
  adjOmegap[101] = Omega[0];
  adjOmegap[125] = 0.0;
  for (b_i = 0; b_i < 6; b_i++) {
    for (vectorUB = 0; vectorUB < 6; vectorUB++) {
      costheta = 0.0;
      for (scalarLB = 0; scalarLB < 6; scalarLB++) {
        costheta += adjOmegap[b_i + 24 * scalarLB] *
                    adjOmegap[scalarLB + 24 * vectorUB];
      }
      b_tmp[b_i + 6 * vectorUB] = costheta;
    }
  }
  for (b_i = 0; b_i < 6; b_i++) {
    for (vectorUB = 0; vectorUB < 6; vectorUB++) {
      adjOmegap[(vectorUB + 24 * b_i) + 6] = b_tmp[vectorUB + 6 * b_i];
    }
  }
  for (b_i = 0; b_i < 6; b_i++) {
    for (vectorUB = 0; vectorUB < 6; vectorUB++) {
      costheta = 0.0;
      for (scalarLB = 0; scalarLB < 6; scalarLB++) {
        costheta += adjOmegap[(b_i + 24 * scalarLB) + 6] *
                    adjOmegap[scalarLB + 24 * vectorUB];
      }
      b_tmp[b_i + 6 * vectorUB] = costheta;
    }
  }
  for (b_i = 0; b_i < 6; b_i++) {
    for (vectorUB = 0; vectorUB < 6; vectorUB++) {
      adjOmegap[(vectorUB + 24 * b_i) + 12] = b_tmp[vectorUB + 6 * b_i];
    }
  }
  for (b_i = 0; b_i < 6; b_i++) {
    for (vectorUB = 0; vectorUB < 6; vectorUB++) {
      costheta = 0.0;
      for (scalarLB = 0; scalarLB < 6; scalarLB++) {
        costheta += adjOmegap[(b_i + 24 * scalarLB) + 12] *
                    adjOmegap[scalarLB + 24 * vectorUB];
      }
      b_tmp[b_i + 6 * vectorUB] = costheta;
    }
  }
  for (b_i = 0; b_i < 6; b_i++) {
    for (vectorUB = 0; vectorUB < 6; vectorUB++) {
      adjOmegap[(vectorUB + 24 * b_i) + 18] = b_tmp[vectorUB + 6 * b_i];
    }
  }
  if (theta <= 0.01) {
    for (b_i = 0; b_i < 16; b_i++) {
      g[b_i] = ((g[b_i] + (real_T)iv[b_i]) + Omegahatp2[b_i] / 2.0) +
               Omegahatp3[b_i] / 6.0;
    }
    f[0] = 0.5;
    f[1] = 0.16666666666666666;
    f[2] = 0.041666666666666664;
    f[3] = 0.0083333333333333332;
    for (b_i = 0; b_i < 6; b_i++) {
      for (vectorUB = 0; vectorUB < 6; vectorUB++) {
        i = vectorUB + 24 * b_i;
        scalarLB = vectorUB + 6 * b_i;
        T[scalarLB] = (((0.5 * adjOmegap[i] + (real_T)iv1[scalarLB]) +
                        0.16666666666666666 * adjOmegap[i + 6]) +
                       0.041666666666666664 * adjOmegap[i + 12]) +
                      0.0083333333333333332 * adjOmegap[i + 18];
      }
    }
  } else {
    real_T sintheta;
    real_T t1;
    real_T t2;
    real_T t3;
    real_T t4;
    real_T tp3;
    Zchp2 = theta * theta;
    tp3 = Zchp2 * theta;
    absxk = tp3 * theta;
    t = absxk * theta;
    sintheta = muDoubleScalarSin(theta);
    costheta = muDoubleScalarCos(theta);
    t1 = theta * sintheta;
    t2 = theta * costheta;
    t3 = ((8.0 - Zchp2) * costheta - 8.0) + 5.0 * t1;
    t4 = (-8.0 * theta + (15.0 - Zchp2) * sintheta) - 7.0 * t2;
    f[0] = ((4.0 - 4.0 * costheta) - t1) / (2.0 * Zchp2);
    f[1] = ((4.0 * theta - 5.0 * sintheta) + t2) / (2.0 * tp3);
    f[2] = ((2.0 - 2.0 * costheta) - t1) / (2.0 * absxk);
    f[3] = ((2.0 * theta - 3.0 * sintheta) + t2) / (2.0 * t);
    fd[0] = t3 / (2.0 * tp3);
    fd[1] = t4 / (2.0 * absxk);
    fd[2] = t3 / (2.0 * t);
    fd[3] = t4 / (2.0 * (t * theta));
    absxk = (1.0 - costheta) / Zchp2;
    Zchp2 = (theta - sintheta) / tp3;
    for (b_i = 0; b_i < 16; b_i++) {
      g[b_i] = ((g[b_i] + (real_T)iv[b_i]) + absxk * Omegahatp2[b_i]) +
               Zchp2 * Omegahatp3[b_i];
    }
    costheta = f[0];
    Zchp2 = f[1];
    absxk = f[2];
    t = f[3];
    for (b_i = 0; b_i < 6; b_i++) {
      for (vectorUB = 0; vectorUB < 6; vectorUB++) {
        i = vectorUB + 24 * b_i;
        scalarLB = vectorUB + 6 * b_i;
        T[scalarLB] = (((costheta * adjOmegap[i] + (real_T)iv1[scalarLB]) +
                        Zchp2 * adjOmegap[i + 6]) +
                       absxk * adjOmegap[i + 12]) +
                      t * adjOmegap[i + 18];
      }
    }
  }
  /*  computation of S */
  st.site = &d_emlrtRSI;
  b_mtimes(T, Z_data, Z_size, S_data, S_size);
}

/* End of code generation (SoftJointKinematics_Z4.c) */
