/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_SoftJointDifferentialKinematics_Z2_api.c
 *
 * Code generation for function '_coder_SoftJointDifferentialKinematics_Z2_api'
 *
 */

/* Include files */
#include "_coder_SoftJointDifferentialKinematics_Z2_api.h"
#include "SoftJointDifferentialKinematics_Z2.h"
#include "SoftJointDifferentialKinematics_Z2_data.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static const mxArray *b_emlrt_marshallOut(const real_T u_data[],
                                          const int32_T u_size[2]);

static real_T *c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                  const char_T *identifier, int32_T y_size[2]);

static const mxArray *c_emlrt_marshallOut(const real_T u[16]);

static real_T *d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                  const emlrtMsgIdentifier *parentId,
                                  int32_T y_size[2]);

static const mxArray *d_emlrt_marshallOut(const real_T u[36]);

static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                   const char_T *identifier))[6];

static const mxArray *e_emlrt_marshallOut(const real_T u[4]);

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier);

static const mxArray *emlrt_marshallOut(const real_T u[6]);

static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[6];

static const mxArray *f_emlrt_marshallOut(const real_T u[144]);

static real_T *g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                  const char_T *identifier, int32_T *y_size);

static real_T *h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                  const emlrtMsgIdentifier *parentId,
                                  int32_T *y_size);

static real_T i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static real_T *j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                  const emlrtMsgIdentifier *msgId,
                                  int32_T ret_size[2]);

static real_T (*k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[6];

static real_T *l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                  const emlrtMsgIdentifier *msgId,
                                  int32_T *ret_size);

/* Function Definitions */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = i_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *b_emlrt_marshallOut(const real_T u_data[],
                                          const int32_T u_size[2])
{
  static const int32_T iv[2] = {0, 0};
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u_data[0]);
  emlrtSetDimensions((mxArray *)m, &u_size[0], 2);
  emlrtAssign(&y, m);
  return y;
}

static real_T *c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                  const char_T *identifier, int32_T y_size[2])
{
  emlrtMsgIdentifier thisId;
  real_T *y_data;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y_data = d_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId, y_size);
  emlrtDestroyArray(&nullptr);
  return y_data;
}

static const mxArray *c_emlrt_marshallOut(const real_T u[16])
{
  static const int32_T iv[2] = {0, 0};
  static const int32_T iv1[2] = {4, 4};
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

static real_T *d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                  const emlrtMsgIdentifier *parentId,
                                  int32_T y_size[2])
{
  real_T *y_data;
  y_data = j_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y_size);
  emlrtDestroyArray(&u);
  return y_data;
}

static const mxArray *d_emlrt_marshallOut(const real_T u[36])
{
  static const int32_T iv[2] = {0, 0};
  static const int32_T iv1[2] = {6, 6};
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                   const char_T *identifier))[6]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[6];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = f_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static const mxArray *e_emlrt_marshallOut(const real_T u[4])
{
  static const int32_T i = 0;
  static const int32_T i1 = 4;
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static const mxArray *emlrt_marshallOut(const real_T u[6])
{
  static const int32_T i = 0;
  static const int32_T i1 = 6;
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[6]
{
  real_T(*y)[6];
  y = k_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *f_emlrt_marshallOut(const real_T u[144])
{
  static const int32_T iv[2] = {0, 0};
  static const int32_T iv1[2] = {24, 6};
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

static real_T *g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                  const char_T *identifier, int32_T *y_size)
{
  emlrtMsgIdentifier thisId;
  real_T *y_data;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y_data = h_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId, y_size);
  emlrtDestroyArray(&nullptr);
  return y_data;
}

static real_T *h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                  const emlrtMsgIdentifier *parentId,
                                  int32_T *y_size)
{
  real_T *y_data;
  y_data = l_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y_size);
  emlrtDestroyArray(&u);
  return y_data;
}

static real_T i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 0U,
                          (const void *)&dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T *j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                  const emlrtMsgIdentifier *msgId,
                                  int32_T ret_size[2])
{
  static const int32_T dims[2] = {6, 100};
  real_T *ret_data;
  int32_T iv[2];
  boolean_T bv[2] = {false, true};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret_size[0] = iv[0];
  ret_size[1] = iv[1];
  ret_data = (real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret_data;
}

static real_T (*k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[6]
{
  static const int32_T dims = 6;
  real_T(*ret)[6];
  int32_T i;
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[6])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T *l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                  const emlrtMsgIdentifier *msgId,
                                  int32_T *ret_size)
{
  static const int32_T dims = 100;
  real_T *ret_data;
  boolean_T b = true;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, ret_size);
  ret_data = (real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret_data;
}

void c_SoftJointDifferentialKinemati(const mxArray *const prhs[6], int32_T nlhs,
                                     const mxArray *plhs[12])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*Phi_Z_data)[600];
  real_T(*S_data)[600];
  real_T(*Sd_data)[600];
  real_T(*Z_data)[600];
  real_T(*dSddq_qd_data)[600];
  real_T(*dSdq_qd_data)[600];
  real_T(*dSdq_qdd_data)[600];
  real_T(*adjOmegap)[144];
  real_T(*q_data)[100];
  real_T(*qd_data)[100];
  real_T(*qdd_data)[100];
  real_T(*T)[36];
  real_T(*g)[16];
  real_T(*Omega)[6];
  real_T(*xi_star_Z)[6];
  real_T(*f)[4];
  real_T(*fd)[4];
  real_T h;
  int32_T Phi_Z_size[2];
  int32_T S_size[2];
  int32_T Sd_size[2];
  int32_T Z_size[2];
  int32_T dSddq_qd_size[2];
  int32_T dSdq_qd_size[2];
  int32_T dSdq_qdd_size[2];
  int32_T q_size;
  int32_T qd_size;
  int32_T qdd_size;
  st.tls = emlrtRootTLSGlobal;
  Omega = (real_T(*)[6])mxMalloc(sizeof(real_T[6]));
  Z_data = (real_T(*)[600])mxMalloc(sizeof(real_T[600]));
  g = (real_T(*)[16])mxMalloc(sizeof(real_T[16]));
  T = (real_T(*)[36])mxMalloc(sizeof(real_T[36]));
  S_data = (real_T(*)[600])mxMalloc(sizeof(real_T[600]));
  Sd_data = (real_T(*)[600])mxMalloc(sizeof(real_T[600]));
  f = (real_T(*)[4])mxMalloc(sizeof(real_T[4]));
  fd = (real_T(*)[4])mxMalloc(sizeof(real_T[4]));
  adjOmegap = (real_T(*)[144])mxMalloc(sizeof(real_T[144]));
  dSdq_qd_data = (real_T(*)[600])mxMalloc(sizeof(real_T[600]));
  dSdq_qdd_data = (real_T(*)[600])mxMalloc(sizeof(real_T[600]));
  dSddq_qd_data = (real_T(*)[600])mxMalloc(sizeof(real_T[600]));
  /* Marshall function inputs */
  h = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "h");
  *(real_T **)&Phi_Z_data =
      c_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "Phi_Z", Phi_Z_size);
  xi_star_Z = e_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "xi_star_Z");
  *(real_T **)&q_data =
      g_emlrt_marshallIn(&st, emlrtAlias(prhs[3]), "q", &q_size);
  *(real_T **)&qd_data =
      g_emlrt_marshallIn(&st, emlrtAlias(prhs[4]), "qd", &qd_size);
  *(real_T **)&qdd_data =
      g_emlrt_marshallIn(&st, emlrtAlias(prhs[5]), "qdd", &qdd_size);
  /* Invoke the target function */
  SoftJointDifferentialKinematics_Z2(
      &st, h, *Phi_Z_data, Phi_Z_size, *xi_star_Z, *q_data, &q_size, *qd_data,
      &qd_size, *qdd_data, &qdd_size, *Omega, *Z_data, Z_size, *g, *T, *S_data,
      S_size, *Sd_data, Sd_size, *f, *fd, *adjOmegap, *dSdq_qd_data,
      dSdq_qd_size, *dSdq_qdd_data, dSdq_qdd_size, *dSddq_qd_data,
      dSddq_qd_size);
  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*Omega);
  if (nlhs > 1) {
    plhs[1] = b_emlrt_marshallOut(*Z_data, Z_size);
  }
  if (nlhs > 2) {
    plhs[2] = c_emlrt_marshallOut(*g);
  }
  if (nlhs > 3) {
    plhs[3] = d_emlrt_marshallOut(*T);
  }
  if (nlhs > 4) {
    plhs[4] = b_emlrt_marshallOut(*S_data, S_size);
  }
  if (nlhs > 5) {
    plhs[5] = b_emlrt_marshallOut(*Sd_data, Sd_size);
  }
  if (nlhs > 6) {
    plhs[6] = e_emlrt_marshallOut(*f);
  }
  if (nlhs > 7) {
    plhs[7] = e_emlrt_marshallOut(*fd);
  }
  if (nlhs > 8) {
    plhs[8] = f_emlrt_marshallOut(*adjOmegap);
  }
  if (nlhs > 9) {
    plhs[9] = b_emlrt_marshallOut(*dSdq_qd_data, dSdq_qd_size);
  }
  if (nlhs > 10) {
    plhs[10] = b_emlrt_marshallOut(*dSdq_qdd_data, dSdq_qdd_size);
  }
  if (nlhs > 11) {
    plhs[11] = b_emlrt_marshallOut(*dSddq_qd_data, dSddq_qd_size);
  }
}

/* End of code generation (_coder_SoftJointDifferentialKinematics_Z2_api.c) */
