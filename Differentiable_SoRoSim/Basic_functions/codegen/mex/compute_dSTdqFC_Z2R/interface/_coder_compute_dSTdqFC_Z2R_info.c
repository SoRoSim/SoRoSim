/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_compute_dSTdqFC_Z2R_info.c
 *
 * Code generation for function 'compute_dSTdqFC_Z2R'
 *
 */

/* Include files */
#include "_coder_compute_dSTdqFC_Z2R_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void);

/* Function Definitions */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void)
{
  const mxArray *nameCaptureInfo;
  const char_T *data[4] = {
      "789cc553cb4ec240141d0c1237206e5cb3704d0c1b1397f2106224a4ed06aca9ed741a46"
      "3b73b19d92ea4f187fc2ef7169e2cf08b48552332911c5bbb93df764"
      "7ace3d934185de750121544151bd97a25e8e7135ee7b68bdb27c41d293da47c5b57309ff"
      "12770c5c905044809b8c2c4fdac02837b9d09e260479c407774aec05",
      "e35097689411350dfa73c43a296a09e6d4fcbb3926f8410d18f2c6fecaa19b06cb3c3e24"
      "fb1637cc2394e451cdf037eddbd6b9ae109f981e1eeb2a38a2a68005"
      "8262bfa601b81684b3a9022a65b54b2aba819540bd451d8778840b6a5a2e3192f185e953"
      "6c3801c78202f7750c6c120862d8aa663f769ac6a8a1d45925e3e3a7",
      "7b1ee5ec99f0f6ec2e19c5606030ed7bcbf4e279e2e36e4b1f25a98f88b1219865f47bf7"
      "3b95eaadf33bb8df6fd1d6597eae871bee297bd76574b0e86faf9f0b"
      "6a577aa727c3c12ef592fa2fbd6ddfe7b144af9ae1475a3f7c1e7adc0a3ac3a079d6f0af"
      "1add5e7be56390a393e70349f05fffff0b07859f94",
      ""};
  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&data[0], 1768U, &nameCaptureInfo);
  return nameCaptureInfo;
}

mxArray *emlrtMexFcnProperties(void)
{
  mxArray *xEntryPoints;
  mxArray *xInputs;
  mxArray *xResult;
  const char_T *propFieldName[9] = {"Version",
                                    "ResolvedFunctions",
                                    "Checksum",
                                    "EntryPoints",
                                    "CoverageInfo",
                                    "IsPolymorphic",
                                    "PropertyList",
                                    "UUID",
                                    "ClassEntryPointIsHandle"};
  const char_T *epFieldName[8] = {
      "Name",     "NumberOfInputs", "NumberOfOutputs", "ConstantInputs",
      "FullPath", "TimeStamp",      "Constructor",     "Visible"};
  xEntryPoints =
      emlrtCreateStructMatrix(1, 1, 8, (const char_T **)&epFieldName[0]);
  xInputs = emlrtCreateLogicalMatrix(1, 7);
  emlrtSetField(xEntryPoints, 0, "Name",
                emlrtMxCreateString("compute_dSTdqFC_Z2R"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(7.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(1.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(
      xEntryPoints, 0, "FullPath",
      emlrtMxCreateString("D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
                          "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
                          "functions\\compute_dSTdqFC_Z2R.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(739686.59887731483));
  emlrtSetField(xEntryPoints, 0, "Constructor",
                emlrtMxCreateLogicalScalar(false));
  emlrtSetField(xEntryPoints, 0, "Visible", emlrtMxCreateLogicalScalar(true));
  xResult =
      emlrtCreateStructMatrix(1, 1, 9, (const char_T **)&propFieldName[0]);
  emlrtSetField(xResult, 0, "Version",
                emlrtMxCreateString("24.1.0.2653294 (R2024a) Update 5"));
  emlrtSetField(xResult, 0, "ResolvedFunctions",
                (mxArray *)c_emlrtMexFcnResolvedFunctionsI());
  emlrtSetField(xResult, 0, "Checksum",
                emlrtMxCreateString("7n7pDLHPmLLVNxUmQsclXG"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

/* End of code generation (_coder_compute_dSTdqFC_Z2R_info.c) */
