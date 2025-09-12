/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * SoftJointKinematics_Z2.c
 *
 * Code generation for function 'SoftJointKinematics_Z2'
 *
 */

/* Include files */
#include "SoftJointKinematics_Z2.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = {
    6,                        /* lineNo */
    "SoftJointKinematics_Z2", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\SoftJointKinematics_Z2.m" /* pathName */
};

static emlrtRSInfo b_emlrtRSI = {
    79,                       /* lineNo */
    "SoftJointKinematics_Z2", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\SoftJointKinematics_Z2.m" /* pathName */
};

static emlrtRSInfo d_emlrtRSI =
    {
        69,                  /* lineNo */
        "eml_mtimes_helper", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pathName */
};

static emlrtRTEInfo emlrtRTEI =
    {
        133,                   /* lineNo */
        23,                    /* colNo */
        "dynamic_size_checks", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2024a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pName */
};

static emlrtRTEInfo b_emlrtRTEI =
    {
        138,                   /* lineNo */
        23,                    /* colNo */
        "dynamic_size_checks", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2024a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pName */
};

/* Function Definitions */
void SoftJointKinematics_Z2(const emlrtStack *sp, real_T h,
                            const real_T Phi_Z_data[],
                            const int32_T Phi_Z_size[2],
                            const real_T xi_star[6], const real_T q_data[],
                            const int32_T q_size[1], real_T Omega[6],
                            real_T Z_data[], int32_T Z_size[2], real_T g[16],
                            real_T T[36], real_T S_data[], int32_T S_size[2],
                            real_T f[4], real_T fd[4], real_T adjOmegap[144])
{
  static const int8_T iv1[36] = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                                 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
  static const int8_T iv[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  __m128d r;
  __m128d r1;
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack st;
  real_T b_adjOmegap[36];
  real_T Omegahatp2[16];
  real_T Omegahatp3[16];
  real_T alpha1;
  real_T beta1;
  real_T costheta;
  real_T t;
  real_T theta;
  int32_T b_i;
  int32_T i;
  int32_T scalarLB;
  int32_T vectorUB;
  char_T TRANSA1;
  char_T TRANSB1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  /* very close to RigidJointKinematics */
  if (q_size[0] == 0) {
    for (i = 0; i < 6; i++) {
      Omega[i] = xi_star[i];
    }
  } else {
    st.site = &emlrtRSI;
    b_st.site = &d_emlrtRSI;
    if (q_size[0] != Phi_Z_size[1]) {
      if (q_size[0] == 1) {
        emlrtErrorWithMessageIdR2018a(
            &b_st, &emlrtRTEI, "Coder:toolbox:mtimes_noDynamicScalarExpansion",
            "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
      } else {
        emlrtErrorWithMessageIdR2018a(&b_st, &b_emlrtRTEI, "MATLAB:innerdim",
                                      "MATLAB:innerdim", 0);
      }
    }
    TRANSB1 = 'N';
    TRANSA1 = 'N';
    alpha1 = 1.0;
    beta1 = 0.0;
    m_t = (ptrdiff_t)6;
    n_t = (ptrdiff_t)1;
    k_t = (ptrdiff_t)Phi_Z_size[1];
    lda_t = (ptrdiff_t)6;
    ldb_t = (ptrdiff_t)q_size[0];
    ldc_t = (ptrdiff_t)6;
    dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1,
          (real_T *)&Phi_Z_data[0], &lda_t, (real_T *)&q_data[0], &ldb_t,
          &beta1, &Omega[0], &ldc_t);
    r = _mm_loadu_pd(&Omega[0]);
    _mm_storeu_pd(&Omega[0], _mm_add_pd(r, _mm_loadu_pd(&xi_star[0])));
    r = _mm_loadu_pd(&Omega[2]);
    _mm_storeu_pd(&Omega[2], _mm_add_pd(r, _mm_loadu_pd(&xi_star[2])));
    r = _mm_loadu_pd(&Omega[4]);
    _mm_storeu_pd(&Omega[4], _mm_add_pd(r, _mm_loadu_pd(&xi_star[4])));
  }
  r = _mm_loadu_pd(&Omega[0]);
  r1 = _mm_set1_pd(h);
  _mm_storeu_pd(&Omega[0], _mm_mul_pd(r1, r));
  r = _mm_loadu_pd(&Omega[2]);
  _mm_storeu_pd(&Omega[2], _mm_mul_pd(r1, r));
  r = _mm_loadu_pd(&Omega[4]);
  _mm_storeu_pd(&Omega[4], _mm_mul_pd(r1, r));
  Z_size[0] = 6;
  Z_size[1] = Phi_Z_size[1];
  i = 6 * Phi_Z_size[1];
  scalarLB = (i / 2) << 1;
  vectorUB = scalarLB - 2;
  for (b_i = 0; b_i <= vectorUB; b_i += 2) {
    _mm_storeu_pd(&Z_data[b_i], _mm_mul_pd(r1, _mm_loadu_pd(&Phi_Z_data[b_i])));
  }
  for (b_i = scalarLB; b_i < i; b_i++) {
    Z_data[b_i] = h * Phi_Z_data[b_i];
  }
  /*     %% defining T, Td, f, fd, fdd, adjOmegap (powers of adjOmega) */
  fd[0] = 0.0;
  fd[1] = 0.0;
  fd[2] = 0.0;
  fd[3] = 0.0;
  memset(&adjOmegap[0], 0, 144U * sizeof(real_T));
  /* all powers of adjGamma || INCLUDE POWER OF 0 to avoid an if */
  alpha1 = 3.3121686421112381E-170;
  beta1 = muDoubleScalarAbs(Omega[0]);
  if (beta1 > 3.3121686421112381E-170) {
    theta = 1.0;
    alpha1 = beta1;
  } else {
    t = beta1 / 3.3121686421112381E-170;
    theta = t * t;
  }
  beta1 = muDoubleScalarAbs(Omega[1]);
  if (beta1 > alpha1) {
    t = alpha1 / beta1;
    theta = theta * t * t + 1.0;
    alpha1 = beta1;
  } else {
    t = beta1 / alpha1;
    theta += t * t;
  }
  beta1 = muDoubleScalarAbs(Omega[2]);
  if (beta1 > alpha1) {
    t = alpha1 / beta1;
    theta = theta * t * t + 1.0;
    alpha1 = beta1;
  } else {
    t = beta1 / alpha1;
    theta += t * t;
  }
  theta = alpha1 * muDoubleScalarSqrt(theta);
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
    alpha1 = Omegahatp2[b_i];
    beta1 = Omegahatp2[b_i + 4];
    t = Omegahatp2[b_i + 8];
    costheta = Omegahatp2[b_i + 12];
    for (vectorUB = 0; vectorUB < 4; vectorUB++) {
      scalarLB = vectorUB << 2;
      Omegahatp3[b_i + scalarLB] =
          ((alpha1 * g[scalarLB] + beta1 * g[scalarLB + 1]) +
           t * g[scalarLB + 2]) +
          costheta * g[scalarLB + 3];
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
      alpha1 = 0.0;
      for (scalarLB = 0; scalarLB < 6; scalarLB++) {
        alpha1 += adjOmegap[b_i + 24 * scalarLB] *
                  adjOmegap[scalarLB + 24 * vectorUB];
      }
      b_adjOmegap[b_i + 6 * vectorUB] = alpha1;
    }
  }
  for (b_i = 0; b_i < 6; b_i++) {
    for (vectorUB = 0; vectorUB < 6; vectorUB++) {
      adjOmegap[(vectorUB + 24 * b_i) + 6] = b_adjOmegap[vectorUB + 6 * b_i];
    }
  }
  for (b_i = 0; b_i < 6; b_i++) {
    for (vectorUB = 0; vectorUB < 6; vectorUB++) {
      alpha1 = 0.0;
      for (scalarLB = 0; scalarLB < 6; scalarLB++) {
        alpha1 += adjOmegap[(b_i + 24 * scalarLB) + 6] *
                  adjOmegap[scalarLB + 24 * vectorUB];
      }
      b_adjOmegap[b_i + 6 * vectorUB] = alpha1;
    }
  }
  for (b_i = 0; b_i < 6; b_i++) {
    for (vectorUB = 0; vectorUB < 6; vectorUB++) {
      adjOmegap[(vectorUB + 24 * b_i) + 12] = b_adjOmegap[vectorUB + 6 * b_i];
    }
  }
  for (b_i = 0; b_i < 6; b_i++) {
    for (vectorUB = 0; vectorUB < 6; vectorUB++) {
      alpha1 = 0.0;
      for (scalarLB = 0; scalarLB < 6; scalarLB++) {
        alpha1 += adjOmegap[(b_i + 24 * scalarLB) + 12] *
                  adjOmegap[scalarLB + 24 * vectorUB];
      }
      b_adjOmegap[b_i + 6 * vectorUB] = alpha1;
    }
  }
  for (b_i = 0; b_i < 6; b_i++) {
    for (vectorUB = 0; vectorUB < 6; vectorUB++) {
      adjOmegap[(vectorUB + 24 * b_i) + 18] = b_adjOmegap[vectorUB + 6 * b_i];
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
    alpha1 = theta * theta;
    tp3 = alpha1 * theta;
    beta1 = tp3 * theta;
    t = beta1 * theta;
    sintheta = muDoubleScalarSin(theta);
    costheta = muDoubleScalarCos(theta);
    t1 = theta * sintheta;
    t2 = theta * costheta;
    t3 = ((8.0 - alpha1) * costheta - 8.0) + 5.0 * t1;
    t4 = (-8.0 * theta + (15.0 - alpha1) * sintheta) - 7.0 * t2;
    f[0] = ((4.0 - 4.0 * costheta) - t1) / (2.0 * alpha1);
    f[1] = ((4.0 * theta - 5.0 * sintheta) + t2) / (2.0 * tp3);
    f[2] = ((2.0 - 2.0 * costheta) - t1) / (2.0 * beta1);
    f[3] = ((2.0 * theta - 3.0 * sintheta) + t2) / (2.0 * t);
    fd[0] = t3 / (2.0 * tp3);
    fd[1] = t4 / (2.0 * beta1);
    fd[2] = t3 / (2.0 * t);
    fd[3] = t4 / (2.0 * (t * theta));
    beta1 = (1.0 - costheta) / alpha1;
    alpha1 = (theta - sintheta) / tp3;
    for (b_i = 0; b_i < 16; b_i++) {
      g[b_i] = ((g[b_i] + (real_T)iv[b_i]) + beta1 * Omegahatp2[b_i]) +
               alpha1 * Omegahatp3[b_i];
    }
    alpha1 = f[0];
    beta1 = f[1];
    t = f[2];
    costheta = f[3];
    for (b_i = 0; b_i < 6; b_i++) {
      for (vectorUB = 0; vectorUB < 6; vectorUB++) {
        i = vectorUB + 24 * b_i;
        scalarLB = vectorUB + 6 * b_i;
        T[scalarLB] = (((alpha1 * adjOmegap[i] + (real_T)iv1[scalarLB]) +
                        beta1 * adjOmegap[i + 6]) +
                       t * adjOmegap[i + 12]) +
                      costheta * adjOmegap[i + 18];
      }
    }
  }
  /*  computation of S */
  st.site = &b_emlrtRSI;
  if (Phi_Z_size[1] == 0) {
    S_size[0] = 6;
    S_size[1] = 0;
  } else {
    TRANSB1 = 'N';
    TRANSA1 = 'N';
    alpha1 = 1.0;
    beta1 = 0.0;
    m_t = (ptrdiff_t)6;
    n_t = (ptrdiff_t)Phi_Z_size[1];
    k_t = (ptrdiff_t)6;
    lda_t = (ptrdiff_t)6;
    ldb_t = (ptrdiff_t)6;
    ldc_t = (ptrdiff_t)6;
    S_size[0] = 6;
    S_size[1] = Phi_Z_size[1];
    dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &T[0], &lda_t,
          &Z_data[0], &ldb_t, &beta1, &S_data[0], &ldc_t);
  }
}

/* End of code generation (SoftJointKinematics_Z2.c) */
